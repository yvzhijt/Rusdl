//! Resistor. Port of `ResistorElm.java`.

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Ideal linear resistor.
///
/// This is the archetype of a pure-conductance stamp and the reason the engine keys nodes
/// off post coordinates: a chain of resistors sharing a post collapses into one node
/// without any explicit union-find in the element layer.
#[derive(Clone, Debug)]
pub struct Resistor {
    base: Base,
    resistance: f64,
}

impl Resistor {
    pub fn from_base(base: Base, resistance: f64) -> Self {
        Self { base, resistance }
    }

    /// Convenience constructor for a horizontal or vertical resistor of `r` ohms.
    pub fn ohm(r: f64) -> Self {
        Self::from_base(Base::new(0, 0, 100, 0, 0, 2), r)
    }

    /// A resistor of `r` ohms spanning `p1`-`p2`.
    pub fn between(p1: Point, p2: Point, r: f64) -> Self {
        Self::from_base(Base::from_points(p1, p2, 0, 2), r)
    }

    pub fn resistance(&self) -> f64 {
        self.resistance
    }

    pub fn set_resistance(&mut self, r: f64) {
        self.resistance = r;
    }

    fn calculate_current(&mut self) {
        self.base.current = self.base.voltage_diff() / self.resistance;
    }
}

impl_element!(
    Resistor,
    posts = 2,
    extra = {
        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            stamper.stamp_resistor(n0, n1, self.resistance);
        }

        /// A resistor is not part of any wire's current reconstruction, so it reports its
        /// current as flowing from post 0 to post 1.
        fn adjust_solved_current(&self, c: f64) -> f64 {
            c
        }
    }
);
