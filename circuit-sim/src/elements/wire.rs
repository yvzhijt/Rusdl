//! Wire. Port of `WireElm.java`.

use crate::element::SimContext;
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// A wire ties its two posts to the same node, so it contributes nothing to the matrix.
///
/// This is a deliberate divergence from a naive MNA implementation, inherited from
/// circuitjs1: instead of stamping a 0V source (which would add two matrix rows per wire
/// and make the matrix badly singular), the engine merges coincident wire endpoints into
/// one node and reconstructs wire currents afterwards from the neighbouring elements'
/// currents. See `Circuit::calc_wire_currents`.
#[derive(Clone, Debug)]
pub struct Wire {
    base: Base,
}

impl Wire {
    pub fn from_base(base: Base) -> Self {
        Self { base }
    }

    /// Wires never compute a current from node voltages; the engine sets it directly.
    fn calculate_current(&mut self) {}
}

impl_element!(
    Wire,
    posts = 2,
    extra = {
        fn is_wire(&self) -> bool {
            true
        }

        /// A `WireElm` *is* the wire, so it is exempt from the voltage-loop check: by the time
        /// that runs, wire endpoints have been merged and the wire is just the wire.
        fn is_wire_like(&self) -> bool {
            false
        }

        fn stamp(&mut self, _stamper: &mut Stamper, _ctx: &SimContext) {}
    }
);

/// Convenience accessor mirroring `WireElm.getVoltageDiff`.
impl Wire {
    pub fn voltage_diff(&self) -> f64 {
        self.base.volts[0]
    }
}
