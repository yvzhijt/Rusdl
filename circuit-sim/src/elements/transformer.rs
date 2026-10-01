//! Ideal transformer.
//!
//! Not a port. circuitjs1 models a transformer as two mutually coupled inductors, and the
//! MNA formulation here has no term for mutual inductance, so there is nothing to port.
//! What is implemented instead is the textbook ideal transformer, which this matrix
//! formulation can express exactly and which is the model a teaching circuit wants: two
//! windings, a turns ratio, and the flux-balance constraint between them.
//!
//! The two constraints are
//!
//! * `V1 = n * V2`, where `n = N1 / N2`, and
//! * `n * I1 + I2 = 0` — ampere-turns balance, i.e. the transformer is lossless.
//!
//! Each becomes one extra matrix row. The first row's unknown is the primary current
//! `I1`; the second row is what couples the windings, and it is why a winding cannot
//! source current on its own. An open secondary simply carries no current. A secondary
//! shorted straight back onto the primary, on the other hand, makes the two rows
//! degenerate into a pair that only has a solution when `n == 1`; that shows up as a
//! singular matrix rather than a silently meaningless answer.

use crate::element::{Element, Point, SimContext};
use crate::elements::common::Base;
use crate::stamper::Stamper;

/// How far the derived secondary winding sits from the primary, in grid units.
///
/// circuitjs1 offsets the secondary by `width` (32 units) on a part 100 units long, so the two
/// windings sit about a third of a span apart. Two cells against a three-cell span is the
/// nearest the integer grid allows, and it keeps the whole symbol from sprawling.
pub const SECONDARY_GAP: i32 = 2;

/// An ideal transformer: two windings with a fixed turns ratio.
///
/// The four posts are two windings of a (plus, minus) pair each: `p1`/`p2` for the primary
/// and `p3`/`p4` for the secondary. Winding polarity is fixed at construction rather than
/// derived from geometry, so a caller rotating the element has to decide for itself which
/// way round each winding faces.
#[derive(Clone, Debug)]
pub struct Transformer {
    base: Base,
    /// Where the secondary's two posts sit, in the same order as the primary's.
    secondary: (Point, Point),
    primary_turns: f64,
    secondary_turns: f64,
    /// The two extra matrix rows, filled in by the engine during analysis.
    vs: [usize; 2],
    /// Current solved for each winding, primary first.
    currents: [f64; 2],
}

impl Transformer {
    /// A transformer with the primary spanning `p1`-`p2` and the secondary spanning
    /// `p3`-`p4`, wound `primary_turns` and `secondary_turns` times.
    pub fn new(
        p1: Point,
        p2: Point,
        p3: Point,
        p4: Point,
        primary_turns: f64,
        secondary_turns: f64,
    ) -> Self {
        Self {
            base: Base::from_points(p1, p2, 0, 4),
            secondary: (p3, p4),
            primary_turns,
            secondary_turns: sanitise_turns(secondary_turns),
            vs: [0, 1],
            currents: [0.0; 2],
        }
    }

    /// Turns ratio `N1 / N2`. A zero secondary is treated as one turn rather than dividing
    /// by zero, which would turn the whole matrix into a NaN.
    pub fn ratio(&self) -> f64 {
        self.primary_turns / sanitise_turns(self.secondary_turns)
    }

    pub fn primary_turns(&self) -> f64 {
        self.primary_turns
    }

    pub fn secondary_turns(&self) -> f64 {
        self.secondary_turns
    }

    pub fn set_turns(&mut self, primary: f64, secondary: f64) {
        self.primary_turns = primary;
        self.secondary_turns = sanitise_turns(secondary);
    }

    /// Voltage across the primary winding, `V(p1) - V(p2)`.
    pub fn primary_voltage(&self) -> f64 {
        self.base.volts[0] - self.base.volts[1]
    }

    /// Voltage across the secondary winding, `V(p3) - V(p4)`.
    pub fn secondary_voltage(&self) -> f64 {
        self.base.volts[2] - self.base.volts[3]
    }

    /// Current in the primary winding, as solved by the engine.
    pub fn primary_current(&self) -> f64 {
        self.currents[0]
    }

    /// Current in the secondary winding, as solved by the engine.
    pub fn secondary_current(&self) -> f64 {
        self.currents[1]
    }

    /// Where a transformer's secondary winding sits when only its primary is placed.
    ///
    /// A transformer needs four terminals but a caller usually has a single span to work
    /// from, so the secondary is derived: the primary's axis rotated a quarter turn and
    /// pushed out by [`SECONDARY_GAP`] cells. Deriving it rather than making the caller
    /// supply four points is what lets a netlist line and a saved canvas describe the same
    /// part, and it is why the winding polarity is fixed — a caller that rotates the part
    /// gets the secondary rotated with it.
    ///
    /// The offset is chosen so the secondary lands on the *right* of a vertical primary and
    /// *above* a horizontal one, which is how the symbol is conventionally drawn.
    pub fn secondary_posts(p1: Point, p2: Point) -> (Point, Point) {
        // The axis is only ever horizontal or vertical: the engine identifies nodes by
        // integer post coordinates, so a diagonal span would not land on the grid.
        let (ux, uy) = if p1.y == p2.y { (1, 0) } else { (0, 1) };
        let offset = Point::new(uy * SECONDARY_GAP, -ux * SECONDARY_GAP);
        (
            Point::new(p1.x + offset.x, p1.y + offset.y),
            Point::new(p2.x + offset.x, p2.y + offset.y),
        )
    }

    /// A transformer whose secondary is placed by [`Transformer::secondary_posts`].
    pub fn with_derived_secondary(
        p1: Point,
        p2: Point,
        primary_turns: f64,
        secondary_turns: f64,
    ) -> Self {
        let (s1, s2) = Self::secondary_posts(p1, p2);
        Self::new(p1, p2, s1, s2, primary_turns, secondary_turns)
    }
}

fn sanitise_turns(turns: f64) -> f64 {
    if turns.is_finite() && turns != 0.0 {
        turns
    } else {
        1.0
    }
}

impl Element for Transformer {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn post_count(&self) -> usize {
        4
    }

    fn post(&self, n: usize) -> Point {
        match n {
            0 => self.base.point1(),
            1 => self.base.point2(),
            2 => self.secondary.0,
            _ => self.secondary.1,
        }
    }

    fn set_node(&mut self, p: usize, n: usize) {
        self.base.nodes[p] = n;
    }

    fn node(&self, n: usize) -> usize {
        self.base.nodes[n]
    }

    fn set_voltage_source(&mut self, n: usize, v: usize) {
        self.vs[n] = v;
    }

    fn set_node_voltage(&mut self, n: usize, v: f64) {
        self.base.volts[n] = v;
    }

    /// The engine hands back the solved value of one extra row, tagged with that row's
    /// *global* index. Comparing against the two indices the engine handed out in
    /// `set_voltage_source` is what tells the two windings apart.
    fn set_current(&mut self, n: usize, c: f64) {
        let slot = if n == self.vs[0] { 0 } else { 1 };
        self.currents[slot] = c;
        // Only the primary drives the reported current, so it is copied rather than left
        // to whichever row happened to be solved last.
        if slot == 0 {
            self.base.current = c;
        }
    }

    fn current(&self) -> f64 {
        self.base.current
    }

    /// Each winding's two posts reported separately, so the current animation marches a phase
    /// down each of the four leads instead of reusing the primary's for all of them. The
    /// secondary current is the load-dependent one: it is whatever the ampere-turns balance
    /// forces, and for an unloaded secondary it is nearly zero.
    fn post_currents(&self) -> Vec<f64> {
        let [primary, secondary] = self.currents;
        vec![-primary, primary, -secondary, secondary]
    }

    fn reset(&mut self) {
        self.base.reset();
        self.currents = [0.0; 2];
    }

    /// One row per winding pair: the voltage relation, then the ampere-turns relation.
    fn voltage_source_count(&self) -> usize {
        2
    }

    /// The windings are d.c.-transparent in this model, so an applied d.c. level reappears
    /// on the secondary scaled by the ratio. That is the ideal-transformer idealisation
    /// rather than the behaviour of a real core, and it is the whole point of using the
    /// ideal model: a resistive secondary gets a well-posed answer with no iteration.
    fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
        let n = self.ratio();
        let (bp, bn, sp, sn) = (
            self.base.nodes[0],
            self.base.nodes[1],
            self.base.nodes[2],
            self.base.nodes[3],
        );
        let (r1, r2) = (
            stamper.node_count() + self.vs[0],
            stamper.node_count() + self.vs[1],
        );

        // Row 1: V(bp) - V(bn) - n * (V(sp) - V(sn)) = 0, unknown I1.
        //
        // This is stamped by hand rather than through `stamp_voltage_source` because that
        // helper can only express a relation between two node voltages, and this one is
        // between two *differences*. The KCL half is the same as any voltage source: the
        // solved row current leaves the positive terminal.
        stamper.stamp_matrix(r1, bp, 1.0);
        stamper.stamp_matrix(r1, bn, -1.0);
        stamper.stamp_matrix(r1, sp, -n);
        stamper.stamp_matrix(r1, sn, n);
        stamper.stamp_matrix(bp, r1, 1.0);
        stamper.stamp_matrix(bn, r1, -1.0);

        // Row 2: n * I1 + I2 = 0, unknown I2.
        stamper.stamp_matrix(r2, r1, n);
        stamper.stamp_matrix(r2, r2, 1.0);
        stamper.stamp_matrix(sp, r2, 1.0);
        stamper.stamp_matrix(sn, r2, -1.0);
    }

    /// Both rows are fully determined before the solve, and neither right-hand side ever
    /// changes, so nothing here needs protecting from matrix simplification.
    fn non_linear(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::{Resistor, VoltageSource};
    use crate::Circuit;

    /// A 2:1 step-down feeding a 1 k load off a 10 V d.c. source. An ideal transformer is
    /// d.c.-transparent, so the load should simply see half the primary voltage and the
    /// primary should see twice the load current.
    #[test]
    fn steps_down_voltage_and_reflects_current() {
        let g = Point::new(0, 0);
        let rail = Point::new(0, 400);
        let sec_hi = Point::new(0, 100);
        let load = Point::new(0, 0);

        let mut c = Circuit::new();
        c.add(Box::new(VoltageSource::dc_between(g, rail, 10.0)));
        c.add(Box::new(Transformer::new(rail, g, sec_hi, load, 2.0, 1.0)));
        c.add(Box::new(Resistor::between(sec_hi, load, 1_000.0)));

        c.analyze().expect("transformer circuit should analyse");
        c.step().expect("transformer circuit should step");

        let t = c.elements()[1]
            .as_any()
            .downcast_ref::<Transformer>()
            .expect("transformer");
        assert!((t.ratio() - 2.0).abs() < 1e-12);
        // 10 V across the primary becomes 5 V across the secondary.
        assert!((t.primary_voltage() - 10.0).abs() < 1e-6);
        assert!((t.secondary_voltage() - 5.0).abs() < 1e-6);
        // 5 V across 1 k is 5 mA, reflected back up as 2.5 mA.
        assert!((t.secondary_current().abs() - 5e-3).abs() < 1e-6);
        assert!((t.primary_current().abs() - 2.5e-3).abs() < 1e-6);
    }

    #[test]
    fn an_open_secondary_draws_no_primary_current() {
        let rail = Point::new(0, 400);
        let g = Point::new(0, 0);
        let mut c = Circuit::new();
        c.add(Box::new(VoltageSource::dc_between(g, rail, 10.0)));
        c.add(Box::new(Resistor::between(rail, g, 1_000.0)));
        c.add(Box::new(Transformer::new(
            rail,
            g,
            Point::new(0, 100),
            Point::new(0, 0),
            2.0,
            1.0,
        )));

        c.analyze().expect("should analyse");
        c.step().expect("should step");

        let t = c.elements()[2]
            .as_any()
            .downcast_ref::<Transformer>()
            .expect("transformer");
        assert!(
            t.primary_current().abs() < 1e-9,
            "primary = {}",
            t.primary_current()
        );
        assert!(t.secondary_current().abs() < 1e-9);
    }

    #[test]
    fn a_zero_secondary_turn_count_does_not_divide_by_zero() {
        let t = Transformer::new(
            Point::new(0, 0),
            Point::new(0, 100),
            Point::new(0, 200),
            Point::new(0, 300),
            4.0,
            0.0,
        );
        assert_eq!(t.secondary_turns(), 1.0);
        assert_eq!(t.ratio(), 4.0);
    }
}
