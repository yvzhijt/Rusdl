//! Potentiometer. Port of `PotElm.java`.
//!
//! Electrically a potentiometer is two resistors in series with the junction brought out as
//! the wiper: `R1 = max * position` between post 0 and the wiper, and
//! `R2 = max * (1 - position)` between the wiper and post 1. That is the whole of it — the
//! zigzag, the arrow and the slider are drawing.
//!
//! The wiper post is *derived* from the two end posts, the way `PotElm.setPoints` derives
//! `post3`, so a netlist that does not name a wiper coordinate still gets a deterministic one.

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Perpendicular offset the wiper sits at, in grid units, when the geometry gives none.
const WIPER_OFFSET: i32 = 16;

/// A three-terminal potentiometer.
#[derive(Clone, Debug)]
pub struct Pot {
    base: Base,
    /// Wiper position, 0..1. 0 puts the wiper at post 1, 1 at post 0.
    position: f64,
    /// Total resistance between the two end posts.
    max_resistance: f64,
    resistance1: f64,
    resistance2: f64,
    current1: f64,
    current2: f64,
    current3: f64,
    wiper: Point,
}

/// How far short of its end of travel a wiper is allowed to get.
///
/// A potentiometer's two resistances are `max * position` and `max * (1 - position)`, so at
/// either extreme one of them is *exactly zero* and the element stamps a zero-resistance
/// branch. The matrix has no way to express that: an ideal zero-ohm element is a voltage
/// constraint, not a conductance, and a conductance of infinity is not a number this solver can
/// factor. The result is not an error the user sees — it is a silent zero, which is worse.
///
/// circuitjs1 has the same stamp and leans on its solver tolerating an infinite entry. This
/// one does not, so the ends of the travel are held just short of zero instead. The gap is
/// small enough that the wiper still reads as being at the end, and the pot still divides.
const TRAVEL_LIMIT: f64 = 1e-4;

fn clamp_position(position: f64) -> f64 {
    if position.is_finite() {
        position.clamp(TRAVEL_LIMIT, 1.0 - TRAVEL_LIMIT)
    } else {
        0.5
    }
}
impl Pot {
    pub fn new(p1: Point, p2: Point, flags: i32, max_resistance: f64, position: f64) -> Self {
        let wiper = Self::wiper_post(p1, p2);
        Self {
            base: Base::from_points(p1, p2, flags, 3),
            position: clamp_position(position),
            max_resistance,
            resistance1: 0.0,
            resistance2: 0.0,
            current1: 0.0,
            current2: 0.0,
            current3: 0.0,
            wiper,
        }
    }

    pub fn from_base(base: Base, max_resistance: f64, position: f64) -> Self {
        let wiper = Self::wiper_post(base.point1(), base.point2());
        Self {
            base,
            position: clamp_position(position),
            max_resistance,
            resistance1: 0.0,
            resistance2: 0.0,
            current1: 0.0,
            current2: 0.0,
            current3: 0.0,
            wiper,
        }
    }

    pub fn position(&self) -> f64 {
        self.position
    }

    pub fn set_position(&mut self, position: f64) {
        self.position = clamp_position(position);
    }

    pub fn max_resistance(&self) -> f64 {
        self.max_resistance
    }

    pub fn set_max_resistance(&mut self, r: f64) {
        self.max_resistance = r;
    }

    /// Resistance between post 0 and the wiper.
    pub fn resistance1(&self) -> f64 {
        self.resistance1
    }

    /// Resistance between the wiper and post 1.
    pub fn resistance2(&self) -> f64 {
        self.resistance2
    }

    /// Port of `PotElm.calculateCurrent`.
    fn calculate_current(&mut self) {
        if self.resistance1 == 0.0 {
            // A zero resistance would make the currents infinite; Java bails out too.
            return;
        }
        let v = &self.base.volts;
        self.current1 = (v[0] - v[2]) / self.resistance1;
        self.current2 = (v[1] - v[2]) / self.resistance2;
        self.current3 = -self.current1 - self.current2;
    }

    /// Where the wiper post sits. Port of `PotElm.post3 = interpPoint(point1, point2, .5, offset)`.
    ///
    /// The offset is the *other* axis's extent, which is what keeps the wiper off the body
    /// whether the pot was drawn horizontally or vertically. A zero-length body falls back to
    /// a fixed offset so the wiper never lands on an end post.
    /// Public because a caller that draws one of these has to know where its terminals are, and
    /// deriving them a second time is how a drawing ends up disagreeing with the element it is
    /// drawing.
    pub fn wiper_post(p1: Point, p2: Point) -> Point {
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        let offset = if dx.abs() > dy.abs() {
            if dx < 0 {
                dy
            } else {
                -dy
            }
        } else if dy != 0 {
            if dy > 0 {
                dx
            } else {
                -dx
            }
        } else {
            0
        };
        let offset = if offset == 0 { WIPER_OFFSET } else { offset };
        let len = ((dx * dx + dy * dy) as f64).sqrt();
        let mx = (p1.x as f64 + p2.x as f64) / 2.0;
        let my = (p1.y as f64 + p2.y as f64) / 2.0;
        if len == 0.0 {
            return Point::new(p1.x, p1.y + WIPER_OFFSET);
        }
        let g = offset as f64 / len;
        Point::new(
            (mx + g * dy as f64 + 0.48).floor() as i32,
            (my - g * dx as f64 + 0.48).floor() as i32,
        )
    }
}

impl_element!(
    Pot,
    posts = 3,
    extra = {
        fn extra_post(&self, n: usize) -> Point {
            let _ = n;
            self.wiper
        }

        /// Port of `PotElm.stamp`.
        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            self.resistance1 = self.max_resistance * self.position;
            self.resistance2 = self.max_resistance * (1.0 - self.position);
            let n0 = self.base.nodes[0];
            let n1 = self.base.nodes[1];
            let n2 = self.base.nodes[2];
            stamper.stamp_resistor(n0, n2, self.resistance1);
            stamper.stamp_resistor(n2, n1, self.resistance2);
        }

        /// Port of `PotElm.getCurrentIntoNode`: each post reports the current flowing out of it,
        /// and the wiper's is the sum of the two resistor currents by KCL.
        fn current_into_node(&self, n: usize) -> f64 {
            match n {
                0 => -self.current1,
                1 => -self.current2,
                _ => -self.current3,
            }
        }
    }
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::Element;

    #[test]
    fn wiper_sits_off_the_body() {
        let p = Pot::new(Point::new(0, 0), Point::new(100, 0), 0, 1_000.0, 0.5);
        assert_eq!(Element::post_count(&p), 3);
        assert_eq!(Element::post(&p, 0), Point::new(0, 0));
        assert_eq!(Element::post(&p, 1), Point::new(100, 0));
        // A horizontal pot offsets the wiper vertically, clear of both end posts.
        let w = Element::post(&p, 2);
        assert_ne!(w, Point::new(0, 0));
        assert_ne!(w, Point::new(100, 0));
    }

    #[test]
    fn resistances_sum_to_the_maximum() {
        let mut p = Pot::new(Point::new(0, 0), Point::new(100, 0), 0, 1_000.0, 0.25);
        let mut stamper = Stamper::new(4, 0);
        let ctx = SimContext::new(1e-3, 0.0, true, 4);
        p.set_node(0, 1);
        p.set_node(1, 2);
        p.set_node(2, 3);
        p.stamp(&mut stamper, &ctx);
        assert!((p.resistance1() - 250.0).abs() < 1e-12);
        assert!((p.resistance2() - 750.0).abs() < 1e-12);
    }
}
