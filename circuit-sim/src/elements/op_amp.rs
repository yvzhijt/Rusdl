//! Operational amplifier. Port of `OpAmpElm.java`.
//!
//! An op-amp is modelled as a voltage-controlled voltage source with a finite gain and a
//! clamped output: `Vout = gain * (V+ - V-)`, limited to `[min_out, max_out]`. The clamp is
//! what makes it nonlinear — near the rails the gain collapses to a tiny value so the output
//! stays put, which is a Newton-friendly way to saturate.
//!
//! The two inputs draw no current and are not connected to each other or to the output, which
//! is why [`OpAmp::get_connection`] is always false. The output *is* connected to ground,
//! through the voltage source that drives it.

use crate::element::{Element, Point, SimContext};
use crate::elements::common::Base;
use crate::stamper::Stamper;

/// Perpendicular distance between the two input posts, in grid units.
const INPUT_OFFSET: f64 = 16.0;
/// Slope the op-amp falls back to in saturation, instead of the full gain.
const SATURATED_SLOPE: f64 = 1e-4;

/// An op-amp: two high-impedance inputs and a clamped, finite-gain output.
#[derive(Clone, Debug)]
pub struct OpAmp {
    base: Base,
    max_out: f64,
    min_out: f64,
    gain: f64,
    lastvd: f64,
    /// `(inverting input, non-inverting input)` post locations.
    inputs: (Point, Point),
}

impl OpAmp {
    pub fn new(p1: Point, p2: Point, flags: i32, max_out: f64, min_out: f64, gain: f64) -> Self {
        let inputs = Self::input_posts(p1, p2);
        Self {
            base: Base::from_points(p1, p2, flags, 3),
            max_out,
            min_out,
            gain,
            lastvd: 0.0,
            inputs,
        }
    }

    pub fn from_base(base: Base, max_out: f64, min_out: f64, gain: f64) -> Self {
        let inputs = Self::input_posts(base.point1(), base.point2());
        Self {
            base,
            max_out,
            min_out,
            gain,
            lastvd: 0.0,
            inputs,
        }
    }

    pub fn max_out(&self) -> f64 {
        self.max_out
    }

    pub fn min_out(&self) -> f64 {
        self.min_out
    }

    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Where the two input posts sit: the inverting input on one side of the body, the
    /// non-inverting on the other, both at the `point1` end. Port of the `in1p[0]`/`in2p[0]`
    /// half of `OpAmpElm.setPoints`.
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

impl Element for OpAmp {
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
        self.lastvd = 0.0;
    }

    /// The output is driven by a voltage source, which is also the only path from the output
    /// to ground. Port of `OpAmpElm.getVoltageSourceCount`.
    fn voltage_source_count(&self) -> usize {
        1
    }

    /// The gain depends on the solution, so the engine switches to the Newton loop.
    fn non_linear(&self) -> bool {
        true
    }

    /// No two terminals of an op-amp are directly connected. Port of
    /// `OpAmpElm.getConnection`.
    fn get_connection(&self, _n1: usize, _n2: usize) -> bool {
        false
    }

    /// Only the output reaches ground, through its voltage source. Port of
    /// `OpAmpElm.hasGroundConnection`.
    fn has_ground_connection(&self, n1: usize) -> bool {
        n1 == 2
    }

    /// Port of `OpAmpElm.stamp`: the output node's KCL references the source row, and that
    /// row's left-hand side is rewritten every iteration.
    fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
        let vn = stamper.node_count() + self.base.volt_source;
        stamper.mark_non_linear(vn);
        stamper.stamp_matrix(self.base.nodes[2], vn, 1.0);
    }

    /// Port of `OpAmpElm.doStep`.
    ///
    /// The Java original consults `lastvd` and a random draw to decide whether to enter
    /// saturation, which is a hysteresis against chattering at the rail. Both are dropped
    /// here: the decision is made from the current `vd` alone, which is deterministic and
    /// behaves the same away from the exact boundary.
    fn do_step(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
        let vd = self.base.volts[1] - self.base.volts[0];
        if (self.lastvd - vd).abs() > 0.1 {
            ctx.set_converged(false);
        } else if self.base.volts[2] > self.max_out + 0.1 || self.base.volts[2] < self.min_out - 0.1
        {
            ctx.set_converged(false);
        }

        let (dx, x) = if vd >= self.max_out / self.gain {
            (
                SATURATED_SLOPE,
                self.max_out - SATURATED_SLOPE * self.max_out / self.gain,
            )
        } else if vd <= self.min_out / self.gain {
            (
                SATURATED_SLOPE,
                self.min_out - SATURATED_SLOPE * self.min_out / self.gain,
            )
        } else {
            (self.gain, 0.0)
        };

        let vn = stamper.node_count() + self.base.volt_source;
        let n0 = self.base.nodes[0];
        let n1 = self.base.nodes[1];
        let n2 = self.base.nodes[2];
        stamper.stamp_matrix(vn, n0, dx);
        stamper.stamp_matrix(vn, n1, -dx);
        stamper.stamp_matrix(vn, n2, 1.0);
        stamper.stamp_right_side(vn, x);

        self.lastvd = vd;
    }

    /// The inputs draw no current; the output reports the source current in the opposite
    /// sense. Port of `OpAmpElm.getCurrentIntoNode`.
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
        let a = OpAmp::new(
            Point::new(0, 0),
            Point::new(100, 0),
            0,
            15.0,
            -15.0,
            100_000.0,
        );
        assert_eq!(Element::post_count(&a), 3);
        let (inv, noninv) = (Element::post(&a, 0), Element::post(&a, 1));
        let out = Element::post(&a, 2);
        assert_ne!(inv, noninv);
        assert_ne!(out, inv);
        assert_ne!(out, noninv);
        // The output is the only terminal that reaches ground.
        assert!(!Element::has_ground_connection(&a, 0));
        assert!(!Element::has_ground_connection(&a, 1));
        assert!(Element::has_ground_connection(&a, 2));
    }
}
