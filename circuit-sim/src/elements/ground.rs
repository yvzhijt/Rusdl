//! Ground. Port of `GroundElm.java`.

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Reference node, modelled as a 0V source to node 0.
///
/// A ground is what forces the whole system to have a definite potential reference: it
/// adds one MNA row, and that row is what makes the matrix non-singular whenever a
/// resistor reaches ground. Note that node 0 is *not* a row in the matrix, so this
/// element's own row is the only thing tying the circuit to the reference.
#[derive(Clone, Debug)]
pub struct Ground {
    base: Base,
}

impl Ground {
    pub fn from_base(base: Base) -> Self {
        Self { base }
    }

    pub fn new(x: i32, y: i32, x2: i32, y2: i32, flags: i32) -> Self {
        Self::from_base(Base::new(x, y, x2, y2, flags, 1))
    }

    pub fn at(x: i32, y: i32) -> Self {
        Self::new(x, y, x, y, 0)
    }

    /// Convenience constructor for a ground sitting on `p`.
    pub fn at_point(p: Point) -> Self {
        Self::at(p.x, p.y)
    }

    fn calculate_current(&mut self) {}
}

impl_element!(
    Ground,
    posts = 1,
    extra = {
        fn voltage_source_count(&self) -> usize {
            1
        }

        fn is_ground(&self) -> bool {
            true
        }

        /// A ground reaches the reference by definition, which is what lets the engine's
        /// ground-closure search terminate on a circuit that has no explicit ground symbol.
        fn has_ground_connection(&self, _n1: usize) -> bool {
            true
        }

        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            let node = self.base.nodes[0];
            let vs = self.base.volt_source;
            stamper.stamp_voltage_source(0, node, vs, 0.0);
        }

        /// The solved variable is the current flowing *out of* the 0V source into the node, so
        /// the conventional ground current (flowing in from the node) is its negation. Port of
        /// `GroundElm.setCurrent`.
        ///
        /// `current_into_node` needs no override: this element has one post rather than two, so
        /// the trait's default already returns `current` unnegated, exactly as circuitjs1 does.
        fn adjust_solved_current(&self, c: f64) -> f64 {
            -c
        }
    }
);
