//! Voltage-controlled analog switch. Port of `AnalogSwitchElm.java`.
//!
//! A resistor between posts 0 and 1 whose value is chosen by the voltage on the control post
//! (post 2): above 2.5 V the switch is open (`r_off`), below it closed (`r_on`). Because the
//! resistance depends on the solution, the element is nonlinear and its stamp is rewritten
//! every iteration.
//!
//! The control post carries no current and is not connected to the switched pair, which is
//! why [`AnalogSwitch::get_connection`] rejects any pair involving post 2.

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// `AnalogSwitchElm.FLAG_INVERT`: a normally-closed switch.
pub const FLAG_INVERT: i32 = 1;
/// Control voltage that separates open from closed, in volts.
const THRESHOLD: f64 = 2.5;
/// Perpendicular offset of the control post from the body, in grid units.
const OPEN_HS: f64 = 16.0;

/// A three-terminal voltage-controlled switch.
#[derive(Clone, Debug)]
pub struct AnalogSwitch {
    base: Base,
    r_on: f64,
    r_off: f64,
    resistance: f64,
    open: bool,
    control: Point,
}

impl AnalogSwitch {
    pub fn new(p1: Point, p2: Point, flags: i32, r_on: f64, r_off: f64) -> Self {
        let control = Self::control_post(p1, p2);
        Self {
            base: Base::from_points(p1, p2, flags, 3),
            r_on,
            r_off,
            resistance: r_off,
            open: true,
            control,
        }
    }

    pub fn from_base(base: Base, r_on: f64, r_off: f64) -> Self {
        let control = Self::control_post(base.point1(), base.point2());
        Self {
            base,
            r_on,
            r_off,
            resistance: r_off,
            open: true,
            control,
        }
    }

    pub fn r_on(&self) -> f64 {
        self.r_on
    }

    pub fn r_off(&self) -> f64 {
        self.r_off
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    /// Port of `AnalogSwitchElm.calculateCurrent`.
    fn calculate_current(&mut self) {
        let v = self.base.voltage_diff();
        self.base.current = v / self.resistance;
    }

    /// Where the control post sits. Port of
    /// `AnalogSwitchElm.point3 = interpPoint(point1, point2, .5, -openhs)`.
    ///
    /// Public because a caller that draws one of these has to know where its terminals are,
    /// and deriving them a second time is how a drawing ends up disagreeing with the element it
    /// is drawing.
    pub fn control_post(p1: Point, p2: Point) -> Point {
        let dx = (p2.x - p1.x) as f64;
        let dy = (p2.y - p1.y) as f64;
        let len = (dx * dx + dy * dy).sqrt();
        let mx = (p1.x as f64 + p2.x as f64) / 2.0;
        let my = (p1.y as f64 + p2.y as f64) / 2.0;
        if len == 0.0 {
            return Point::new(p1.x, p1.y - 16);
        }
        let g = -OPEN_HS / len;
        Point::new(
            (mx + g * dy + 0.48).floor() as i32,
            (my - g * dx + 0.48).floor() as i32,
        )
    }
}

impl_element!(
    AnalogSwitch,
    posts = 3,
    extra = {
        fn extra_post(&self, n: usize) -> Point {
            let _ = n;
            self.control
        }

        /// The resistance depends on the solution, so the whole circuit becomes nonlinear and
        /// the engine switches to the Newton loop. Port of `AnalogSwitchElm.nonLinear`.
        fn non_linear(&self) -> bool {
            true
        }

        /// The control post is a sense terminal: it carries no current and is not connected to
        /// the switched pair. Port of `AnalogSwitchElm.getConnection`.
        fn get_connection(&self, n1: usize, n2: usize) -> bool {
            n1 != 2 && n2 != 2
        }

        /// Port of `AnalogSwitchElm.stamp`: only the rows that get rewritten every iteration
        /// need protecting from simplification.
        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            stamper.mark_non_linear(n0);
            stamper.mark_non_linear(n1);
        }

        /// Port of `AnalogSwitchElm.doStep`: pick the resistance from the control voltage.
        fn do_step(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            let mut open = self.base.volts[2] < THRESHOLD;
            if self.base.flags & FLAG_INVERT != 0 {
                open = !open;
            }
            self.open = open;
            self.resistance = if open { self.r_off } else { self.r_on };
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            stamper.stamp_resistor(n0, n1, self.resistance);
        }

        /// Port of `AnalogSwitchElm.getCurrentIntoNode`: the control post carries none.
        fn current_into_node(&self, n: usize) -> f64 {
            match n {
                0 => -self.base.current,
                1 => self.base.current,
                _ => 0.0,
            }
        }
    }
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::Element;

    #[test]
    fn control_post_is_distinct_and_unconnected() {
        let s = AnalogSwitch::new(Point::new(0, 0), Point::new(100, 0), 0, 20.0, 1e10);
        assert_eq!(Element::post_count(&s), 3);
        let c = Element::post(&s, 2);
        assert_ne!(c, Point::new(0, 0));
        assert_ne!(c, Point::new(100, 0));
        // The control terminal is a sense node, not part of the switched path.
        assert!(!Element::get_connection(&s, 0, 2));
        assert!(!Element::get_connection(&s, 1, 2));
        assert!(Element::get_connection(&s, 0, 1));
    }

    #[test]
    fn threshold_picks_the_resistance() {
        let mut s = AnalogSwitch::new(Point::new(0, 0), Point::new(100, 0), 0, 20.0, 1e10);
        let mut stamper = Stamper::new(4, 0);
        let ctx = SimContext::new(1e-3, 0.0, true, 4);
        s.set_node(0, 1);
        s.set_node(1, 2);
        s.set_node(2, 3);

        // Control below the threshold: open.
        s.set_node_voltage(2, 0.0);
        s.do_step(&mut stamper, &ctx);
        assert!(s.is_open());
        assert_eq!(s.resistance, 1e10);

        // Control above the threshold: closed.
        s.set_node_voltage(2, 5.0);
        s.do_step(&mut stamper, &ctx);
        assert!(!s.is_open());
        assert_eq!(s.resistance, 20.0);
    }
}
