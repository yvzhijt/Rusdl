//! Voltage source. Port of `VoltageElm.java`.
//!
//! `RailElm` in circuitjs1 is just a `VoltageElm` with a different symbol and a flag
//! marking it as a supply rail, which is why `is_rail` is separate from `is_voltage_source`.

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::expression::Expr;
use crate::stamper::Stamper;
use std::f64::consts::PI;

/// Source waveform. Values match `VoltageElm`'s `WF_*` constants so netlist flags decode
/// without a translation table.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Waveform {
    #[default]
    Dc,
    Ac,
    Square,
    Triangle,
    Sawtooth,
    Pulse,
    Noise,
    Var,
    /// Whatever the source's [`crate::expression::Expr`] says. Not a circuitjs1 waveform: the
    /// value comes from the caller's expression in the simulation time, so the amplitude,
    /// frequency, bias, phase and duty settings are all ignored.
    Custom,
}

impl Waveform {
    /// The netlist flag for this waveform, matching `VoltageElm`'s `WF_*` constants.
    ///
    /// `Custom` has no circuitjs1 equivalent, so it takes the next free code rather than
    /// colliding with one that means something else.
    pub fn code(self) -> i32 {
        match self {
            Self::Dc => 0,
            Self::Ac => 1,
            Self::Square => 2,
            Self::Triangle => 3,
            Self::Sawtooth => 4,
            Self::Pulse => 5,
            Self::Noise => 6,
            Self::Var => 7,
            Self::Custom => 8,
        }
    }

    pub fn from_code(code: i32) -> Option<Self> {
        Some(match code {
            0 => Self::Dc,
            1 => Self::Ac,
            2 => Self::Square,
            3 => Self::Triangle,
            4 => Self::Sawtooth,
            5 => Self::Pulse,
            6 => Self::Noise,
            7 => Self::Var,
            8 => Self::Custom,
            _ => return None,
        })
    }

    /// Whether the source's value has to be recomputed every timestep.
    fn is_time_varying(self) -> bool {
        self != Self::Dc
    }
}

/// Port of `VoltageElm.triangleFunc`.
fn triangle_func(x: f64) -> f64 {
    if x < PI {
        x * (2.0 / PI) - 1.0
    } else {
        1.0 - (x - PI) * (2.0 / PI)
    }
}

/// Independent voltage source, optionally time-varying.
///
/// A DC source stamps its value directly into the right-hand side once, so the engine can
/// factor the matrix a single time and reuse it for the whole run. Any time-varying
/// waveform instead marks its row as dynamic and rewrites the value in every `do_step`,
/// which is why those circuits cannot skip the per-timestep refactorisation.
#[derive(Clone, Debug)]
pub struct VoltageSource {
    base: Base,
    waveform: Waveform,
    frequency: f64,
    max_voltage: f64,
    freq_time_zero: f64,
    bias: f64,
    phase_shift: f64,
    duty_cycle: f64,
    noise_value: f64,
    is_rail: bool,
    /// The expression a `Custom` waveform evaluates. Meaningless for every other waveform,
    /// and kept as an `Option` so the common case costs nothing.
    custom: Option<Expr>,
    /// Set when a `Custom` expression produced a value that is not a number, so the run can
    /// be stopped with an explanation instead of the solver diverging on a NaN.
    custom_failed: bool,
}

impl VoltageSource {
    #[allow(clippy::too_many_arguments)]
    pub fn from_base(
        base: Base,
        waveform: Waveform,
        frequency: f64,
        max_voltage: f64,
        bias: f64,
        phase_shift: f64,
        duty_cycle: f64,
        is_rail: bool,
    ) -> Self {
        Self {
            base,
            waveform,
            frequency,
            max_voltage,
            freq_time_zero: 0.0,
            bias,
            phase_shift,
            duty_cycle,
            noise_value: 0.0,
            is_rail,
            custom: None,
            custom_failed: false,
        }
    }

    /// Convenience constructor for a DC source of `volts`.
    pub fn dc(volts: f64) -> Self {
        Self::from_base(
            Base::new(0, 0, 0, 100, 0, 2),
            Waveform::Dc,
            0.0,
            volts,
            0.0,
            0.0,
            0.0,
            false,
        )
    }

    /// A DC source of `volts` spanning `p1`-`p2`, with `p2` as the positive terminal.
    ///
    /// The polarity is not a free choice: `stamp_voltage_source` writes
    /// `V(node(p2)) - V(node(p1)) = volts`, inherited verbatim from `stampVoltageSource`, so
    /// `p2` is where the higher potential appears.
    pub fn dc_between(p1: Point, p2: Point, volts: f64) -> Self {
        Self::from_base(
            Base::from_points(p1, p2, 0, 2),
            Waveform::Dc,
            0.0,
            volts,
            0.0,
            0.0,
            0.0,
            false,
        )
    }

    /// Convenience constructor for a sine source of `volts` peak at `hz`.
    pub fn sine(hz: f64, volts: f64) -> Self {
        Self::from_base(
            Base::new(0, 0, 0, 100, 0, 2),
            Waveform::Ac,
            hz,
            volts,
            0.0,
            0.0,
            0.0,
            false,
        )
    }

    /// A source whose value at time `t` is whatever `expr` says, in volts.
    ///
    /// Convenience over `from_base` plus two setters, and the reason `set_custom` exists as a
    /// single call: setting the waveform and the expression separately would leave a window
    /// in which the source claims a custom shape but has nothing to evaluate.
    pub fn custom(base: Base, expr: Expr) -> Self {
        let mut source = Self::from_base(base, Waveform::Custom, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        source.custom = Some(expr);
        source
    }

    pub fn waveform(&self) -> Waveform {
        self.waveform
    }

    pub fn set_waveform(&mut self, wf: Waveform) {
        self.waveform = wf;
    }

    pub fn max_voltage(&self) -> f64 {
        self.max_voltage
    }

    pub fn set_max_voltage(&mut self, v: f64) {
        self.max_voltage = v;
    }

    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    pub fn set_frequency(&mut self, f: f64) {
        self.frequency = f;
    }

    pub fn bias(&self) -> f64 {
        self.bias
    }

    pub fn set_bias(&mut self, b: f64) {
        self.bias = b;
    }

    pub fn phase_shift(&self) -> f64 {
        self.phase_shift
    }

    pub fn set_phase_shift(&mut self, p: f64) {
        self.phase_shift = p;
    }

    pub fn duty_cycle(&self) -> f64 {
        self.duty_cycle
    }

    pub fn set_duty_cycle(&mut self, d: f64) {
        self.duty_cycle = d;
    }

    /// The expression a `Custom` waveform evaluates, if one has been set.
    pub fn custom_expression(&self) -> Option<&Expr> {
        self.custom.as_ref()
    }

    /// Sets the expression for a `Custom` waveform, and switches the source over to it.
    ///
    /// Passing `None` leaves the source on whatever waveform it had, since a source with no
    /// expression is a source the caller has not finished describing.
    pub fn set_custom(&mut self, expr: Option<Expr>) {
        if expr.is_some() {
            self.waveform = Waveform::Custom;
        }
        self.custom = expr;
        self.custom_failed = false;
    }

    /// Whether a `Custom` expression has ever produced a value that is not a number.
    ///
    /// `1/t` is a perfectly good expression that is only undefined at `t = 0`, so the failure
    /// cannot be caught when the expression is set; this is how the caller finds out.
    pub fn custom_failed(&self) -> bool {
        self.custom_failed
    }

    /// Source value at simulation time `t`. Port of `VoltageElm.getVoltage`.
    ///
    /// A `Custom` expression is in volts on its own: it is not scaled by the amplitude, and
    /// the frequency, bias, phase and duty settings do not apply to it, because the whole
    /// point is that the expression *is* the waveform.
    pub fn voltage_at(&self, t: f64, dc_analysis: bool) -> f64 {
        if self.waveform.is_time_varying() && dc_analysis {
            return self.bias;
        }
        if self.waveform == Waveform::Custom {
            return self.custom.as_ref().map_or(f64::NAN, |expr| expr.eval(t));
        }
        let w = 2.0 * PI * (t - self.freq_time_zero) * self.frequency + self.phase_shift;
        match self.waveform {
            Waveform::Dc => self.max_voltage + self.bias,
            Waveform::Ac => w.sin() * self.max_voltage + self.bias,
            Waveform::Square => {
                let hi = if w.rem_euclid(2.0 * PI) > 2.0 * PI * self.duty_cycle {
                    -self.max_voltage
                } else {
                    self.max_voltage
                };
                self.bias + hi
            }
            Waveform::Triangle => {
                self.bias + triangle_func(w.rem_euclid(2.0 * PI)) * self.max_voltage
            }
            Waveform::Sawtooth => {
                self.bias + (w.rem_euclid(2.0 * PI)) * (self.max_voltage / PI) - self.max_voltage
            }
            Waveform::Pulse => {
                if w.rem_euclid(2.0 * PI) < 2.0 * PI * self.duty_cycle {
                    self.max_voltage + self.bias
                } else {
                    self.bias
                }
            }
            Waveform::Noise => self.noise_value,
            Waveform::Var => 0.0,
            // Handled before the match, since a custom source ignores every setting below.
            Waveform::Custom => 0.0,
        }
    }

    /// The solved variable is the current out of the positive terminal.
    fn calculate_current(&mut self) {}
}

impl_element!(
    VoltageSource,
    posts = 2,
    extra = {
        fn voltage_source_count(&self) -> usize {
            1
        }

        fn is_voltage_source(&self) -> bool {
            true
        }

        fn is_rail(&self) -> bool {
            self.is_rail
        }

        /// A source with a DC level is a valid anchor for the ground-closure search.
        fn has_ground_connection(&self, _n1: usize) -> bool {
            false
        }

        fn stamp(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            let vs = self.base.volt_source;
            if self.waveform == Waveform::Dc {
                let v = self.voltage_at(ctx.time, ctx.dc_analysis);
                stamper.stamp_voltage_source(n0, n1, vs, v);
            } else {
                stamper.stamp_voltage_source_dynamic(n0, n1, vs);
            }
        }

        fn do_step(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            if self.waveform == Waveform::Dc {
                return;
            }
            let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
            let vs = self.base.volt_source;
            let v = self.voltage_at(ctx.time, ctx.dc_analysis);
            // A custom expression can be undefined at one particular time — `1/t` at zero, a
            // square root of a negative — and stamping a NaN would poison the solve in a way
            // that surfaces much later as a singular matrix. Zero is stamped so the timestep
            // is merely wrong, the flag lets the caller explain, and the run stops.
            if !v.is_finite() {
                self.custom_failed = true;
                ctx.request_stop();
                stamper.update_voltage_source(n0, n1, vs, 0.0);
                return;
            }
            stamper.update_voltage_source(n0, n1, vs, v);
        }

        fn step_finished(&mut self, _ctx: &SimContext) {
            if self.waveform == Waveform::Noise {
                self.noise_value = (rand01() * 2.0 - 1.0) * self.max_voltage + self.bias;
            }
        }

        /// The MNA unknown is the current flowing into post 0, but the conventional reported
        /// source current is the one delivered out of the positive terminal (post 1), so the sign
        /// flips here exactly as `VoltageElm.setCurrent` does.
        ///
        /// `get_connection` is deliberately *not* overridden: circuitjs1's `CircuitElm` default is
        /// `true` and `VoltageElm` does not override it, which is what lets the voltage-loop
        /// search walk *through* a source to find the loop.
        fn adjust_solved_current(&self, c: f64) -> f64 {
            -c
        }
    }
);

impl VoltageSource {
    /// Voltage across the source, `V(1) - V(0)`, the opposite sense to other elements.
    pub fn voltage_diff(&self) -> f64 {
        self.base.volts[1] - self.base.volts[0]
    }
}

/// Small xorshift PRNG. The Java original uses `java.util.Random`; reproducibility of the
/// exact noise sequence is not important, but determinism across runs is, so we keep a
/// seeded generator rather than thread-local state.
fn rand01() -> f64 {
    use std::cell::Cell;
    thread_local! {
        static STATE: Cell<u64> = const { Cell::new(0x2545F4914F6CDD1D) };
    }
    STATE.with(|s| {
        let mut x = s.get();
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        s.set(x);
        (x >> 11) as f64 / (1u64 << 53) as f64
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::{Ground, Resistor};
    use crate::Circuit;

    fn source_from(text: &str) -> VoltageSource {
        VoltageSource::custom(
            Base::new(0, 0, 0, 100, 0, 2),
            Expr::parse(text).unwrap_or_else(|err| panic!("{text:?}: {err}")),
        )
    }

    #[test]
    fn a_custom_expression_is_the_source_voltage() {
        let source = source_from("3 - sin(t - pi/2)");
        // 3 - sin(t - pi/2) is 3 + cos(t).
        assert!((source.voltage_at(0.0, false) - 4.0).abs() < 1e-12);
        assert!((source.voltage_at(std::f64::consts::PI, false) - 2.0).abs() < 1e-12);
        // Not scaled by an amplitude and not offset by a bias: the expression is the value,
        // which is the whole point of choosing this waveform.
        assert_eq!(source.max_voltage(), 0.0);
        assert_eq!(source.bias(), 0.0);
        assert_eq!(source.waveform(), Waveform::Custom);
    }

    #[test]
    fn a_custom_source_without_an_expression_is_not_silently_zero_volts() {
        let mut source = source_from("1");
        source.set_custom(None);
        assert!(!source.voltage_at(0.0, false).is_finite());
    }

    #[test]
    fn every_waveform_code_round_trips() {
        for waveform in [
            Waveform::Dc,
            Waveform::Ac,
            Waveform::Square,
            Waveform::Triangle,
            Waveform::Sawtooth,
            Waveform::Pulse,
            Waveform::Noise,
            Waveform::Var,
            Waveform::Custom,
        ] {
            assert_eq!(Waveform::from_code(waveform.code()), Some(waveform));
        }
    }

    #[test]
    fn a_custom_source_drives_a_load_like_any_other() {
        // 5 * sin(2*pi*50*t) into a 1 k load, so the load voltage should peak at 5 V and the
        // run should cover several cycles.
        let mut circuit = Circuit::new();
        circuit.add(Box::new(source_from("5*sin(2*pi*50*t)")));
        circuit.add(Box::new(Resistor::ohm(1_000.0)));
        circuit.add(Box::new(Ground::at(0, 100)));
        circuit.analyze().expect("analyze");

        let mut peak = 0.0_f64;
        let mut rising_edges = 0;
        let mut previous = 0.0_f64;
        // Enough steps to cover several cycles: the engine's step is at most 5 us, and at
        // 50 Hz a single cycle is 20 ms, so four thousand steps is only one of them.
        for _ in 0..50_000 {
            circuit.step().expect("step");
            let source = circuit.elements()[0]
                .as_any()
                .downcast_ref::<VoltageSource>()
                .expect("the source");
            let voltage = source.voltage_diff();
            peak = peak.max(voltage.abs());
            if previous < 0.0 && voltage >= 0.0 {
                rising_edges += 1;
            }
            previous = voltage;
        }
        assert!(peak > 4.9, "peak = {peak}, expected the 5 V amplitude");
        assert!(
            rising_edges >= 3,
            "the run should cover several cycles, got {rising_edges}"
        );
    }

    #[test]
    fn an_undefined_expression_stops_the_run_instead_of_poisoning_the_solve() {
        // 1/t is a perfectly good expression that is only undefined at t = 0, so neither the
        // parser nor the setter can catch it.
        let mut circuit = Circuit::new();
        circuit.add(Box::new(source_from("1/t")));
        circuit.add(Box::new(Resistor::ohm(1_000.0)));
        circuit.add(Box::new(Ground::at(0, 100)));
        circuit.analyze().expect("analyze");
        // The first step lands on t = 0, where the expression has no value.
        let _ = circuit.step();
        let failed = circuit
            .elements()
            .iter()
            .filter_map(|element| element.as_any().downcast_ref::<VoltageSource>())
            .any(|source| source.custom_failed());
        assert!(failed, "the source should have flagged the failure");
    }
}
