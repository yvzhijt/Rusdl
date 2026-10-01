//! SPDT switch. Port of `Switch2Elm.java`.
//!
//! An SPDT switch connects its pole (post 0) to one of `throw_count` throws, or to none at
//! all in the centre-off position. Electrically it is the same 0 V source trick as
//! [`Switch`]: the selected throw is shorted to the pole and the unselected throws are left
//! floating.
//!
//! The throw posts are *derived* from the pole and the reference point, exactly as
//! `Switch2Elm.setPoints` does, so a netlist that places a throw at a given coordinate means
//! the same thing here as it does in circuitjs1.

use crate::element::{Element, Point, SimContext};
use crate::elements::common::Base;
use crate::stamper::Stamper;

/// `Switch2Elm.FLAG_CENTER_OFF`: with two throws, position 2 disconnects everything.
pub const FLAG_CENTER_OFF: i32 = 1;
/// Perpendicular distance, in grid units, from the reference point to the outer throws.
const OPEN_HS: f64 = 16.0;

/// A single-pole, double-throw (or multi-throw) switch.
#[derive(Clone, Debug)]
pub struct SpdtSwitch {
    base: Base,
    /// Which throw the pole is connected to, or 2 for centre-off.
    position: u8,
    momentary: bool,
    /// Switches sharing a link toggle together. A UI concern; see [`SpdtSwitch::toggle`].
    link: i32,
    throw_count: usize,
    throw_posts: Vec<Point>,
}

impl SpdtSwitch {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        p1: Point,
        p2: Point,
        flags: i32,
        position: u8,
        momentary: bool,
        link: i32,
        throw_count: usize,
    ) -> Self {
        let throw_posts = Self::throw_post_layout(p1, p2, throw_count);
        Self {
            base: Base::from_points(p1, p2, flags, 1 + throw_count),
            position,
            momentary,
            link,
            throw_count,
            throw_posts,
        }
    }

    pub fn from_base(
        base: Base,
        position: u8,
        momentary: bool,
        link: i32,
        throw_count: usize,
    ) -> Self {
        let throw_posts = Self::throw_post_layout(base.point1(), base.point2(), throw_count);
        Self {
            base,
            position,
            momentary,
            link,
            throw_count,
            throw_posts,
        }
    }

    /// A two-throw switch with the pole at `p1`.
    pub fn between(p1: Point, p2: Point) -> Self {
        Self::new(p1, p2, 0, 0, false, 0, 2)
    }

    pub fn position(&self) -> u8 {
        self.position
    }

    pub fn throw_count(&self) -> usize {
        self.throw_count
    }

    pub fn is_momentary(&self) -> bool {
        self.momentary
    }

    pub fn link(&self) -> i32 {
        self.link
    }

    /// `Switch2Elm.hasCenterOff`: centre-off only exists for a two-throw switch.
    pub fn has_center_off(&self) -> bool {
        self.throw_count == 2 && self.base.flags & FLAG_CENTER_OFF != 0
    }

    /// True when the switch is in the centre-off position, connecting nothing.
    pub fn is_center_off(&self) -> bool {
        self.position == 2 && self.has_center_off()
    }

    /// Number of positions the switch cycles through.
    fn position_count(&self) -> usize {
        if self.has_center_off() {
            3
        } else {
            self.throw_count
        }
    }

    /// Port of `Switch2Elm.toggle`.
    ///
    /// The Java original also copies the new position to every other switch sharing the same
    /// `link`, which is how a DPDT switch is built from two SPDTs. That needs a whole
    /// circuit to walk, so it is left to the caller; [`crate::Circuit`] has no hook for it.
    pub fn toggle(&mut self) {
        self.position = ((self.position as usize + 1) % self.position_count()) as u8;
    }

    /// Port of `Switch2Elm.calculateCurrent`: centre-off carries nothing.
    fn calculate_current(&mut self) {
        if self.is_center_off() {
            self.base.current = 0.0;
        }
    }

    /// Where each throw post sits. Port of the `swposts` half of `Switch2Elm.setPoints`.
    ///
    /// Public because a caller that draws one of these has to know where its terminals are,
    /// and deriving them a second time is how a drawing ends up disagreeing with the element
    /// it is drawing.
    pub fn throw_post_layout(p1: Point, p2: Point, throw_count: usize) -> Vec<Point> {
        let gx = (p2.y - p1.y) as f64;
        let gy = (p1.x - p2.x) as f64;
        let norm = (gx * gx + gy * gy).sqrt();
        // Java's `(throwCount-1)/2` is integer division.
        let mid = (throw_count - 1) / 2;
        (0..throw_count)
            .map(|i| {
                let mut hs = -OPEN_HS * (i as f64 - mid as f64);
                if throw_count == 2 && i == 0 {
                    hs = OPEN_HS;
                }
                // `interpPoint(point1, point2, c, 1, hs)`, with the degenerate case of a
                // zero-length axis falling back to no offset at all.
                let g = if norm > 0.0 { hs / norm } else { 0.0 };
                Point::new(
                    (p2.x as f64 + g * gx + 0.48).floor() as i32,
                    (p2.y as f64 + g * gy + 0.48).floor() as i32,
                )
            })
            .collect()
    }
}

impl Element for SpdtSwitch {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn post_count(&self) -> usize {
        1 + self.throw_count
    }

    fn post(&self, n: usize) -> Point {
        if n == 0 {
            self.base.point1()
        } else {
            self.throw_posts[n - 1]
        }
    }

    fn set_node(&mut self, p: usize, n: usize) {
        self.base.nodes[p] = n;
    }

    fn node(&self, n: usize) -> usize {
        self.base.nodes[n]
    }

    fn set_voltage_source(&mut self, _n: usize, v: usize) {
        self.base.volt_source = v;
    }

    fn set_node_voltage(&mut self, n: usize, v: f64) {
        self.base.volts[n] = v;
        self.calculate_current();
    }

    fn set_current(&mut self, _n: usize, c: f64) {
        let c = self.adjust_solved_current(c);
        self.base.current = c;
    }

    fn current(&self) -> f64 {
        self.base.current
    }

    fn reset(&mut self) {
        self.base.reset();
    }

    /// Centre-off contributes no row. Port of `Switch2Elm.getVoltageSourceCount`.
    fn voltage_source_count(&self) -> usize {
        usize::from(!self.is_center_off())
    }

    /// Only the selected throw is connected to the pole. Port of
    /// `Switch2Elm.getConnection`, which compares the pair against `(0, position+1)`.
    fn get_connection(&self, n1: usize, n2: usize) -> bool {
        if self.is_center_off() {
            return false;
        }
        let a = 0;
        let b = 1 + self.position as usize;
        (n1 == a && n2 == b) || (n1 == b && n2 == a)
    }

    /// `Switch2Elm.isWire` is unconditionally true, so an SPDT is always traversable by the
    /// path searches — even in centre-off, where `get_connection` says otherwise.
    fn is_wire_like(&self) -> bool {
        true
    }

    /// Port of `Switch2Elm.stamp`.
    fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
        if self.is_center_off() {
            return;
        }
        let n0 = self.base.nodes[0];
        let n1 = self.base.nodes[1 + self.position as usize];
        stamper.stamp_voltage_source(n0, n1, self.base.volt_source, 0.0);
    }

    /// Port of `Switch2Elm.getCurrentIntoNode`: the pole and the connected throw carry the
    /// current in opposite senses, and the unselected throws carry none at all.
    fn current_into_node(&self, n: usize) -> f64 {
        if n == 0 {
            -self.current()
        } else if n == 1 + self.position as usize {
            self.current()
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn throw_posts_are_distinct_and_symmetric() {
        let s = SpdtSwitch::between(Point::new(0, 0), Point::new(100, 0));
        assert_eq!(Element::post_count(&s), 3);
        // A horizontal element offsets the throws vertically, one up and one down.
        assert_eq!(s.post(0), Point::new(0, 0));
        assert_eq!(s.post(1), Point::new(100, -16));
        assert_eq!(s.post(2), Point::new(100, 16));
    }

    #[test]
    fn only_the_selected_throw_is_connected() {
        let mut s = SpdtSwitch::between(Point::new(0, 0), Point::new(100, 0));
        assert!(Element::get_connection(&s, 0, 1));
        assert!(!Element::get_connection(&s, 0, 2));

        s.toggle();
        assert!(!Element::get_connection(&s, 0, 1));
        assert!(Element::get_connection(&s, 0, 2));
    }

    #[test]
    fn centre_off_disconnects_everything() {
        let s = SpdtSwitch::new(
            Point::new(0, 0),
            Point::new(100, 0),
            FLAG_CENTER_OFF,
            2,
            false,
            0,
            2,
        );
        assert!(s.is_center_off());
        assert_eq!(Element::voltage_source_count(&s), 0);
        assert!(!Element::get_connection(&s, 0, 1));
        assert!(!Element::get_connection(&s, 0, 2));
        // Centre-off is only reachable on a two-throw switch.
        assert_eq!(s.position_count(), 3);
    }
}
