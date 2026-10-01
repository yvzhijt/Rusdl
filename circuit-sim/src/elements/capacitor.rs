//! Capacitor. Port of `CapacitorElm.java`.

use crate::element::SimContext;
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Set when the trapezoidal approximation should be disabled in favour of backward Euler.
pub const FLAG_BACK_EULER: i32 = 2;

/// Ideal capacitor, integrated with a companion (Norton) model.
///
/// The capacitor is linear, but its companion model depends on the timestep, so the engine
/// has to re-stamp the whole system whenever the timestep changes. Both ends mark their
/// right-hand side as dynamic, which also keeps matrix simplification from folding the
/// row away.
///
/// Trapezoidal integration is the default because it is second-order accurate, but it
/// rings when RC is small relative to the timestep, so backward Euler is available.
#[derive(Clone, Debug)]
pub struct Capacitor {
    base: Base,
    capacitance: f64,
    /// Voltage across the capacitor at the start of the iteration, `V(0) - V(1)`.
    voltdiff: f64,
    /// Value stamped by `start_iteration` and injected by `do_step`.
    cur_source_value: f64,
    /// `dt / (2C)` (trapezoidal) or `dt / C` (backward Euler). Set by `stamp`.
    comp_resistance: f64,
    /// Small voltage pre-loaded on reset so oscillators start.
    initial_voltage: f64,
}

impl Capacitor {
    pub fn from_base(base: Base, capacitance: f64) -> Self {
        let initial_voltage = 1e-3;
        Self {
            base,
            capacitance,
            voltdiff: initial_voltage,
            cur_source_value: 0.0,
            comp_resistance: 0.0,
            initial_voltage,
        }
    }

    /// Convenience constructor for a capacitor of `farads`.
    pub fn farad(capacitance: f64) -> Self {
        Self::from_base(Base::new(0, 0, 100, 0, 0, 2), capacitance)
    }

    pub fn capacitance(&self) -> f64 {
        self.capacitance
    }

    pub fn set_capacitance(&mut self, c: f64) {
        self.capacitance = c;
    }

    pub fn is_trapezoidal(&self) -> bool {
        self.base.flags & FLAG_BACK_EULER == 0
    }

    pub fn set_trapezoidal(&mut self, on: bool) {
        if on {
            self.base.flags &= !FLAG_BACK_EULER;
        } else {
            self.base.flags |= FLAG_BACK_EULER;
        }
    }

    pub fn voltage_diff(&self) -> f64 {
        self.base.voltage_diff()
    }

    /// Initial voltage the capacitor is charged to at reset. Also sets the live `voltdiff`
    /// so the value takes effect even if [`Circuit::reset`](crate::Circuit::reset) is never
    /// called before the first step.
    pub fn initial_voltage(&self) -> f64 {
        self.initial_voltage
    }

    pub fn set_initial_voltage(&mut self, v: f64) {
        self.initial_voltage = v;
        self.voltdiff = v;
    }

    /// Tracked so the trapezoidal rule can use the previous iteration's voltage.
    fn calculate_current(&mut self) {
        let v = self.base.voltage_diff();
        self.voltdiff = v;
        // `comp_resistance` is checked because `calculateCurrent` can be reached from
        // `setNodeVoltage` during circuit analysis, before `stamp` has run.
        if self.comp_resistance > 0.0 {
            self.base.current = v / self.comp_resistance + self.cur_source_value;
        }
    }
}

impl_element!(
    Capacitor,
    posts = 2,
    extra = {
        fn is_capacitor(&self) -> bool {
            true
        }

        fn reset_extra(&mut self) {
            self.cur_source_value = 0.0;
            // Pre-load a little charge so oscillators have something to start from.
            self.voltdiff = self.initial_voltage;
        }

        /// Told by the engine that a short was found across this capacitor. Dropping the
        /// stored voltage is what stops a shorted pair from oscillating forever.
        fn shorted(&mut self) {
            self.base.reset();
            self.voltdiff = 0.0;
            self.cur_source_value = 0.0;
        }

        fn stamp(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            if ctx.dc_analysis {
                // When finding the DC operating point a capacitor is an open circuit.
                stamper.stamp_resistor(n0, n1, 1e8);
                self.cur_source_value = 0.0;
                return;
            }
            self.comp_resistance = if self.is_trapezoidal() {
                ctx.time_step / (2.0 * self.capacitance)
            } else {
                ctx.time_step / self.capacitance
            };
            stamper.stamp_resistor(n0, n1, self.comp_resistance);
            // Both right-hand sides are rewritten every iteration, so mark them dynamic.
            stamper.mark_rhs_changes(n0);
            stamper.mark_rhs_changes(n1);
        }

        fn start_iteration(&mut self, ctx: &SimContext) {
            if ctx.dc_analysis {
                return;
            }
            self.cur_source_value = if self.is_trapezoidal() {
                -self.voltdiff / self.comp_resistance - self.base.current
            } else {
                -self.voltdiff / self.comp_resistance
            };
        }

        fn do_step(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            if ctx.dc_analysis {
                return;
            }
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            stamper.stamp_current_source(n0, n1, self.cur_source_value);
        }
    }
);
