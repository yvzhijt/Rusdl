//! Element implementations.
//!
//! The set here is deliberately the core of the engine's vocabulary — the devices needed
//! to exercise every path through the solver: linear stamps, dynamic right-hand sides,
//! companion models for the reactive elements, an extra matrix row for voltage sources,
//! and a genuinely nonlinear device that drives the Newton loop. The remaining ~120
//! circuitjs1 elements are UI-heavy specialisations of these same five stamp patterns.
//!
//! Seven of them — [`Switch`], [`SpdtSwitch`], [`AnalogSwitch`], [`Pot`], [`OpAmp`],
//! [`Comparator`] and [`CurrentSource`] — are in the desktop editor's palette as of the shelf
//! that groups it by active, passive, switched and instrument. They are reachable from a
//! netlist through [`crate::parse_netlist`] as well.

mod analog_switch;
mod capacitor;
pub(crate) mod common;
mod comparator;
mod current_source;
mod diode;
mod ground;
mod inductor;
mod op_amp;
mod pot;
mod resistor;
mod spdt_switch;
mod switch;
mod transformer;
mod transistor;
mod voltage_source;
mod wire;

pub use common::Base;

pub use analog_switch::AnalogSwitch;
pub use capacitor::Capacitor;
pub use comparator::Comparator;
pub use current_source::CurrentSource;
pub use diode::{builtin_model, Diode, DiodeKind, DiodeModel, FLAG_FWDROP, FLAG_MODEL};
pub use ground::Ground;
pub use inductor::Inductor;
pub use op_amp::OpAmp;
pub use pot::Pot;
pub use resistor::Resistor;
pub use spdt_switch::SpdtSwitch;
pub use switch::Switch;
pub use transformer::Transformer;
pub use transistor::Bjt;
pub use voltage_source::{VoltageSource, Waveform};
pub use wire::Wire;
