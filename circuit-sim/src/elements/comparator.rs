//! Comparator. Port of `ComparatorElm.java`.
//!
//! CircuitJS builds a comparator as a *composite* — an op-amp whose output drives an analog
//! switch pulling the output terminal to ground — which is an open-drain output. This port
//! keeps the behaviour and drops the internal structure: the output is tied to ground through
//! a low resistance when `V+ > V-` and a very high one otherwise, so an external pull-up
//! turns it into a comparator.

use crate::element::{Element, Point, SimContext};
use crate::elements::common::Base;
use crate::stamper::Stamper;

/// Perpendicular distance between the two input posts, in grid units.
const INPUT_OFFSET: f64 = 16.0;
/// Resistance from the output to ground when `V+ > V-`.
const R_ON: f64 = 20.0;
/// Resistance from the output to ground when `V+ <= V-`.
const R_OFF: f64 = 1e10;

/// An open-drain comparator: two high-impedance inputs and an output that pulls to ground
/// when `V+ > V-`.
#[derive(Clone, Debug)]
pub struct Comparator {
    base: Base,
    /// `(inverting input, non-inverting input)` post locations.
    inputs: (Point, Point),
}

impl Comparator {
    pub fn new(p1: Point, p2: Point, flags: i32) -> Self {
        let inputs = Self::input_posts(p1, p2);
        Self {
            base: Base::from_points(p1, p2, flags, 3),
            inputs,
        }
    }

    pub fn from_base(base: Base) -> Self {
        let inputs = Self::input_posts(base.point1(), base.point2());
        Self { base, inputs }
    }

    /// Port of `ComparatorElm.calculateCurrent`: the current through the output-to-ground
    /// element, which is the only current the comparator carries.
    fn calculate_current(&mut self) {
        let r = self.output_resistance();
        self.base.current = self.base.volts[2] / r;
    }

    /// Resistance from the output to ground, chosen by the input difference.
    fn output_resistance(&self) -> f64 {
        if self.base.volts[1] - self.base.volts[0] > 0.0 {
            R_ON
        } else {
            R_OFF
        }
    }

    /// Where the two input posts sit. Port of the `in1p[0]`/`in2p[0]` half of
    /// `ComparatorElm.setPoints`, which is the same layout the op-amp uses.
    ///
    /// Public because a caller that draws one of these has to know where its terminals are,
    /// and deriving them a second time is how a drawing ends up disagreeing with the element
    /// it is drawing.
    pub fn input_posts(p1: Point, p2: Point) -> (Point, Point) {
        let dx = (p2.x - p1.x) as f64;
        let dy = (p2.y - p1.y) as f64;
        let len = (dx * dx + dy * dy).sqrt();
        if len == 0.0 {
            return (Point::new(p1.x, p1.y + 16), Point::new(p1.x, p1.y - 16));
        }
        let g = INPUT_OFFSET / len;
        let ox = g * dy;
        let oy = -g * dx;
        (
            Point::new(
                (p1.x as f64 + ox + 0.48).floor() as i32,
                (p1.y as f64 + oy + 0.48).floor() as i32,
            ),
            Point::new(
                (p1.x as f64 - ox + 0.48).floor() as i32,
                (p1.y as f64 - oy + 0.48).floor() as i32,
            ),
        )
    }
}

impl Element for Comparator {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn post_count(&self) -> usize {
        3
    }

    fn post(&self, n: usize) -> Point {
        match n {
            0 => self.inputs.0,
            1 => self.inputs.1,
            _ => self.base.point2(),
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

    /// The output-to-ground resistance depends on the solution, so the engine switches to the
    /// Newton loop.
    fn non_linear(&self) -> bool {
        true
    }

    /// No two terminals are directly connected. Port of `ComparatorElm.getConnection`.
    fn get_connection(&self, _n1: usize, _n2: usize) -> bool {
        false
    }

    /// The output reaches ground through the pull-down. Port of the composite's
    /// `GroundElm 5` at the far end of the analog switch.
    fn has_ground_connection(&self, n1: usize) -> bool {
        n1 == 2
    }

    /// The output row is rewritten every iteration.
    fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
        stamper.mark_non_linear(self.base.nodes[2]);
    }

    /// Port of the composite's `doStep`: pick the pull-down resistance from the input
    /// difference.
    fn do_step(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
        let r = self.output_resistance();
        stamper.stamp_resistor(self.base.nodes[2], 0, r);
    }

    /// The inputs draw no current; the output reports the pull-down current in the opposite
    /// sense.
    fn current_into_node(&self, n: usize) -> f64 {
        if n == 2 {
            -self.base.current
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inputs_and_output_are_distinct() {
        let c = Comparator::new(Point::new(0, 0), Point::new(100, 0), 0);
        assert_eq!(Element::post_count(&c), 3);
        let (inv, noninv) = (Element::post(&c, 0), Element::post(&c, 1));
        let out = Element::post(&c, 2);
        assert_ne!(inv, noninv);
        assert_ne!(out, inv);
        assert_ne!(out, noninv);
        assert!(!Element::has_ground_connection(&c, 0));
        assert!(!Element::has_ground_connection(&c, 1));
        assert!(Element::has_ground_connection(&c, 2));
    }

    #[test]
    fn output_resistance_follows_the_input_difference() {
        let mut c = Comparator::new(Point::new(0, 0), Point::new(100, 0), 0);
        let mut stamper = Stamper::new(4, 0);
        let ctx = SimContext::new(1e-3, 0.0, true, 4);
        c.set_node(0, 1);
        c.set_node(1, 2);
        c.set_node(2, 3);

        // V+ above V-: the output pulls to ground through 20 ohms.
        c.set_node_voltage(0, 0.0);
        c.set_node_voltage(1, 5.0);
        c.do_step(&mut stamper, &ctx);
        assert_eq!(c.output_resistance(), R_ON);

        // V+ below V-: the pull-down releases.
        c.set_node_voltage(1, 0.0);
        c.do_step(&mut stamper, &ctx);
        assert_eq!(c.output_resistance(), R_OFF);
    }
}
