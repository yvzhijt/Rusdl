//! Inductor. Port of `InductorElm.java` plus the `Inductor` companion model it wraps.

use crate::element::SimContext;
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Set when the trapezoidal approximation should be disabled in favour of backward Euler.
pub const FLAG_BACK_EULER: i32 = 2;

/// Ideal inductor, integrated with a companion (Norton) model.
///
/// Structurally identical to [`crate::elements::Capacitor`] but with `2L/dt` (trapezoidal)
/// or `L/dt` (backward Euler) in place of `dt/(2C)` or `dt/C`. The resistor is stamped
/// across the two nodes and the current source is injected on top of it.
///
/// The `cur_source_value` is deliberately initialised to zero in [`Inductor::reset`]:
/// when one terminal is node 0 the engine can call `set_node_voltage` during analysis,
/// before `start_iteration` has ever run.
#[derive(Clone, Debug)]
pub struct Inductor {
    base: Base,
    inductance: f64,
    comp_resistance: f64,
    cur_source_value: f64,
}

impl Inductor {
    pub fn from_base(base: Base, inductance: f64, current: f64) -> Self {
        Self {
            base,
            inductance,
            comp_resistance: 0.0,
            cur_source_value: 0.0,
        }
        .with_current(current)
    }

    /// Convenience constructor for an inductor of `henries`.
    pub fn henry(inductance: f64) -> Self {
        Self::from_base(Base::new(0, 0, 100, 0, 0, 2), inductance, 0.0)
    }

    fn with_current(mut self, current: f64) -> Self {
        self.base.current = current;
        self
    }

    pub fn inductance(&self) -> f64 {
        self.inductance
    }

    pub fn set_inductance(&mut self, l: f64) {
        self.inductance = l;
    }

    /// Initial current through the inductor, as stored in a netlist. Named apart from the
    /// [`Element`](crate::Element) trait's `current`/`set_current` to avoid shadowing them.
    pub fn initial_current(&self) -> f64 {
        self.base.current
    }

    pub fn set_initial_current(&mut self, current: f64) {
        self.base.current = current;
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

    fn calculate_current(&mut self) {
        let v = self.base.voltage_diff();
        // Guard against `calculate_current` running before `stamp` has set
        // `comp_resistance`.
        if self.comp_resistance > 0.0 {
            self.base.current = v / self.comp_resistance + self.cur_source_value;
        }
    }
}

impl_element!(
    Inductor,
    posts = 2,
    extra = {
        fn is_inductor(&self) -> bool {
            true
        }

        fn reset_extra(&mut self) {
            self.cur_source_value = 0.0;
        }

        fn stamp(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            self.comp_resistance = if self.is_trapezoidal() {
                2.0 * self.inductance / ctx.time_step
            } else {
                self.inductance / ctx.time_step
            };
            stamper.stamp_resistor(n0, n1, self.comp_resistance);
            stamper.mark_rhs_changes(n0);
            stamper.mark_rhs_changes(n1);
        }

        fn start_iteration(&mut self, ctx: &SimContext) {
            if ctx.dc_analysis {
                return;
            }
            let v = self.base.voltage_diff();
            self.cur_source_value = if self.is_trapezoidal() {
                v / self.comp_resistance + self.base.current
            } else {
                self.base.current
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
