//! Small, school-level circuit instruments.
//!
//! The solver also supports circuitjs1-compatible elements, but a basic circuit only needs
//! [`PowerSupply`], resistors, wires, ground and (when studying transients) capacitors.
//! The meters live together here and do not add another layer to the solver.
//!
//! There is deliberately no oscilloscope. A caller that wants a trace reads the meters in a
//! loop — the desktop editor does exactly that, sampling on a fixed interval of *simulated*
//! time rather than once per solver step, since the solver's adaptive timestep moves over six
//! orders of magnitude and a per-step sample would give a window anywhere from nanoseconds to
//! milliseconds wide. Keeping a sample buffer in the engine would only have offered a second,
//! worse place to keep that policy.

use crate::circuit::Circuit;
use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::elements::VoltageSource;
use crate::stamper::Stamper;
use std::fmt;

/// A DC source; use [`PowerSupply::dc_between`] with the positive terminal second.
pub type PowerSupply = VoltageSource;

/// Error returned when a meter is connected to a point that is not part of the circuit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InstrumentError {
    MissingPoint(Point),
}

impl fmt::Display for InstrumentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingPoint(p) => write!(f, "no circuit terminal at ({}, {})", p.x, p.y),
        }
    }
}

impl std::error::Error for InstrumentError {}

fn voltage_at(circuit: &Circuit, point: Point) -> Result<f64, InstrumentError> {
    let node = circuit
        .node_at_point(point)
        .ok_or(InstrumentError::MissingPoint(point))?;
    Ok(circuit.node_voltage(node))
}

/// An ideal voltmeter. It measures positive terminal minus negative terminal and does not
/// change the circuit.
#[derive(Clone, Copy, Debug)]
pub struct Voltmeter {
    positive: Point,
    negative: Point,
}

impl Voltmeter {
    pub fn between(positive: Point, negative: Point) -> Self {
        Self { positive, negative }
    }

    pub fn read(&self, circuit: &Circuit) -> Result<f64, InstrumentError> {
        Ok(voltage_at(circuit, self.positive)? - voltage_at(circuit, self.negative)?)
    }
}

/// An ideal ammeter, placed in series. Positive current is from the first point to the
/// second point.
#[derive(Clone, Debug)]
pub struct Ammeter {
    base: Base,
}

impl Ammeter {
    pub fn between(first: Point, second: Point) -> Self {
        Self {
            base: Base::from_points(first, second, 0, 2),
        }
    }

    /// Current in amperes after a successful simulation step.
    pub fn amperes(&self) -> f64 {
        self.base.current
    }

    fn calculate_current(&mut self) {}
}

impl_element!(
    Ammeter,
    posts = 2,
    extra = {
        fn voltage_source_count(&self) -> usize {
            1
        }

        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            stamper.stamp_voltage_source(
                self.base.nodes[0],
                self.base.nodes[1],
                self.base.volt_source,
                0.0,
            );
        }
    }
);
