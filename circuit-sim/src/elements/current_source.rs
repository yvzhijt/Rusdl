//! Current source. Port of `CurrentElm.java`.

use crate::element::SimContext;
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Independent current source.
///
/// A current source whose terminals have no path to ground would make the matrix
/// singular, because there is no reference for the injected charge to go against. The
/// engine's validation pass detects this and calls [`Element::set_broken`]; the element
/// then stamps a large resistor instead and reports zero current.
#[derive(Clone, Debug)]
pub struct CurrentSource {
    base: Base,
    value: f64,
    broken: bool,
}

impl CurrentSource {
    pub fn from_base(base: Base, value: f64) -> Self {
        Self {
            base,
            value,
            broken: false,
        }
    }

    /// Convenience constructor for a source of `amps`.
    pub fn amps(amps: f64) -> Self {
        Self::from_base(Base::new(0, 0, 100, 0, 0, 2), amps)
    }

    pub fn value(&self) -> f64 {
        self.value
    }

    pub fn set_value(&mut self, v: f64) {
        self.value = v;
    }

    /// The engine forces the reported current to the source value; the solved variable
    /// for a current source row is unused.
    fn calculate_current(&mut self) {
        if !self.broken {
            self.base.current = self.value;
        }
    }
}

impl_element!(
    CurrentSource,
    posts = 2,
    extra = {
        fn is_current_source(&self) -> bool {
            true
        }

        fn set_broken(&mut self, broken: bool) {
            self.broken = broken;
            if broken {
                self.base.current = 0.0;
            }
        }

        fn reset_extra(&mut self) {
            self.broken = false;
            self.calculate_current();
        }

        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            if self.broken {
                // No current path; a current source here would cause a matrix error.
                stamper.stamp_resistor(n0, n1, 1e8);
                self.base.current = 0.0;
            } else {
                stamper.stamp_current_source(n0, n1, self.value);
                self.base.current = self.value;
            }
        }

        /// A source's reported current drives the wire-current reconstruction, and it flows
        /// from post 1 into post 0, hence the sign flip relative to the resistor.
        fn adjust_solved_current(&self, c: f64) -> f64 {
            -c
        }
    }
);
