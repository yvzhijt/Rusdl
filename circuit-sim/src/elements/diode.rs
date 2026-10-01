//! Diode, Zener, and LED. Port of `Diode.java`, `DiodeElm.java`, `ZenerElm.java`,
//! `LEDElm.java`, and the model side of `DiodeModel.java`.
//!
//! The diode exists to exercise the parts of the engine that a linear circuit never
//! reaches: a genuinely solution-dependent stamp, the Newton loop in `Circuit::step`, the
//! convergence flag, and the `gmin` conductance ramp that rescues a struggling solve.
//!
//! The three Java element classes differ *only* in which [`DiodeModel`] they are handed and
//! what they are called in a netlist — the physics is one code path. Zener adds a reverse
//! breakdown branch, which lives in the model rather than the element, and LED differs from
//! the default diode mainly by having a series resistance. So all three are one Rust type
//! ([`Diode`]) with a [`DiodeKind`] tag, rather than three near-identical element types.

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Electron thermal voltage at SPICE's default 27 C (300.15 K).
const VT: f64 = 0.025865;
/// Multiplicative equivalent of dividing by [`VT`], precomputed as in the Java original.
const VZCOEF: f64 = 1.0 / VT;
/// The `DiodeElm` flag bit that replaces the next netlist token with a model name.
pub const FLAG_MODEL: i32 = 2;
/// The `DiodeElm` flag bit that replaces the next netlist token with a forward voltage drop.
pub const FLAG_FWDROP: i32 = 1;

/// User-visible diode parameters. Port of `DiodeModel`.
///
/// These are the SPICE parameters the curve is actually built from. The familiar
/// "forward voltage" figure is *derived* from them, not stored — see
/// [`DiodeModel::forward_voltage_drop`]. CircuitJS deliberately exposes the derived
/// number as the user-facing knob and back-solves `saturation_current` from it, but the
/// engine only ever integrates the stored fields.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeModel {
    /// Reverse saturation current `Is`, the current scale of the Shockley curve.
    pub saturation_current: f64,
    /// Ideality (emission) coefficient `n`. The e-folding voltage is `n * vt`.
    pub emission_coefficient: f64,
    /// Reverse breakdown voltage `Vz`. Zero disables Zener behaviour.
    pub breakdown_voltage: f64,
    /// Series resistance `Rs`. Zero means the junction is stamped directly between the two
    /// posts; anything else adds a hidden node and a real resistor, which is what makes an
    /// LED 3-terminal in the model even though it is drawn with two leads.
    pub series_resistance: f64,
}

impl DiodeModel {
    /// Voltage drop at 1 A. Derived, never stored — this is CircuitJS's `fwdrop`.
    pub fn forward_voltage_drop(&self) -> f64 {
        (1.0 / self.saturation_current + 1.0).ln() * self.emission_coefficient * VT
    }

    /// Builds the model CircuitJS would get by asking for a given forward drop.
    ///
    /// The drop is converted back into a saturation current, since the curve is
    /// parameterised by `Is`. Port of `DiodeModel.getModelWithParameters`, which is still
    /// the path a netlist takes when it carries no model name.
    pub fn with_forward_voltage_drop(forward_voltage_drop: f64, emission_coefficient: f64) -> Self {
        let vscale = emission_coefficient * VT;
        let leakage = 1.0 / ((forward_voltage_drop / vscale).exp() - 1.0);
        Self {
            saturation_current: leakage,
            emission_coefficient,
            breakdown_voltage: 0.0,
            series_resistance: 0.0,
        }
    }
}

impl Default for DiodeModel {
    /// The `spice-default` model: the textbook `Is = 1e-14`, `n = 1`.
    fn default() -> Self {
        Self {
            saturation_current: 1e-14,
            emission_coefficient: 1.0,
            breakdown_voltage: 0.0,
            series_resistance: 0.0,
        }
    }
}

/// The built-in model set, ported from `DiodeModel.createModelMap`.
///
/// CircuitJS keeps these in a mutable global map that a running simulation can extend with
/// user-created models. A headless engine has no model editor, so the table is a plain
/// function and a name that is not in it falls back to [`DiodeModel::default`] — which is
/// also what `getModelWithNameOrCopy` does when it cannot find a name and has nothing to
/// copy.
pub fn builtin_model(name: &str) -> Option<DiodeModel> {
    let m = |saturation_current, series_resistance, emission_coefficient, breakdown_voltage| {
        Some(DiodeModel {
            saturation_current,
            series_resistance,
            emission_coefficient,
            breakdown_voltage,
        })
    };
    match name {
        // The literal SPICE default: too ideal to be useful, kept for compatibility.
        "spice-default" => m(1e-14, 0.0, 1.0, 0.0),
        // The default CircuitJS picks for a plain diode, back-fitted to a ~0.8 V drop.
        "default" => m(1.7143528192808883e-7, 0.0, 2.0, 0.0),
        "default-zener" => m(1.7143528192808883e-7, 0.0, 2.0, 5.6),
        // Superseded by `default-led`; its saturation current is so small that the
        // Newton loop cannot resolve it. Kept so old netlists still load.
        "old-default-led" => m(2.2349907006671927e-18, 0.0, 2.0, 0.0),
        "default-led" => m(93.2e-12, 0.042, 3.73, 0.0),
        "1N5711" => m(315e-9, 2.8, 2.03, 70.0),
        "1N5712" => m(680e-12, 12.0, 1.003, 20.0),
        "1N34" => m(200e-12, 84e-3, 2.19, 60.0),
        "1N4004" => m(18.8e-9, 28.6e-3, 2.0, 400.0),
        "1N4148" => m(4.352e-9, 0.6458, 1.906, 75.0),
        _ => None,
    }
}

/// Which of the three Java subclasses an element is, for netlist and UI purposes.
///
/// The engine treats all three identically; only the dump type and the default model differ.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DiodeKind {
    /// `DiodeElm`, dump type `'d'`.
    #[default]
    Plain,
    /// `ZenerElm`, dump type `'z'`.
    Zener,
    /// `LEDElm`, dump type `162`.
    Led,
}

impl DiodeKind {
    /// The one-character-or-number type a netlist uses for this element.
    pub fn dump_type(&self) -> i32 {
        match self {
            Self::Plain => 'd' as i32,
            Self::Zener => 'z' as i32,
            Self::Led => 162,
        }
    }
}

/// Derived, cached form of [`DiodeModel`]. Port of the fields in `Diode`, plus the series
/// resistor that `DiodeElm` owns (the Java comment on `Diode` says so explicitly).
#[derive(Clone, Debug)]
pub struct Diode {
    base: Base,
    kind: DiodeKind,
    leakage: f64,
    fwdrop: f64,
    zvoltage: f64,
    series_resistance: f64,
    vscale: f64,
    vdcoef: f64,
    vcrit: f64,
    vzcrit: f64,
    zoffset: f64,
    lastvoltdiff: f64,
}

impl Diode {
    /// A diode between two points, with an explicit model.
    pub fn new(p1: Point, p2: Point, flags: i32, model: &DiodeModel, kind: DiodeKind) -> Self {
        let base = Base::from_points(p1, p2, flags, model.node_slots());
        Self::from_base(base, model, kind)
    }

    /// A plain diode with a given forward drop at 1 A. Port of the plain `DiodeElm`
    /// netlist constructor, whose `fwdrop` token is only read when [`FLAG_FWDROP`] is set.
    pub fn with_forward_voltage_drop(fwdrop: f64) -> Self {
        let model = DiodeModel::with_forward_voltage_drop(fwdrop, 2.0);
        Self::new(
            Point::new(0, 0),
            Point::new(100, 0),
            0,
            &model,
            DiodeKind::Plain,
        )
    }

    /// A 5.6 V Zener, the CircuitJS default. Port of `ZenerElm`'s `default_zvoltage`.
    pub fn zener(p1: Point, p2: Point, flags: i32, zvoltage: f64) -> Self {
        let model = DiodeModel {
            // The Zener keeps the default diode's curve and only adds the breakdown term.
            ..DiodeModel::with_forward_voltage_drop(0.805904783, 2.0)
        }
        .with_breakdown_voltage(zvoltage);
        Self::new(p1, p2, flags, &model, DiodeKind::Zener)
    }

    /// An LED with CircuitJS's default model (`93.2pA`, `n = 3.73`, `42 mΩ` series).
    pub fn led(p1: Point, p2: Point, flags: i32) -> Self {
        let model = builtin_model("default-led").expect("built-in model");
        Self::new(p1, p2, flags, &model, DiodeKind::Led)
    }

    pub fn from_base(base: Base, model: &DiodeModel, kind: DiodeKind) -> Self {
        let mut d = Self {
            base,
            kind,
            leakage: 0.0,
            fwdrop: 0.0,
            zvoltage: 0.0,
            series_resistance: 0.0,
            vscale: 0.0,
            vdcoef: 0.0,
            vcrit: 0.0,
            vzcrit: 0.0,
            zoffset: 0.0,
            lastvoltdiff: 0.0,
        };
        d.apply_model(model);
        d
    }

    /// Which of the three netlist types this element is.
    pub fn kind(&self) -> DiodeKind {
        self.kind
    }

    /// Recomputes the cached constants derived from `model`. Port of `Diode.setModel`.
    ///
    /// Updates the derived fields in place rather than rebuilding the element, so the node
    /// numbers the engine assigned survive a model change. Port of `Diode.setup`.
    fn apply_model(&mut self, model: &DiodeModel) {
        let leakage = model.saturation_current;
        let zvoltage = model.breakdown_voltage;
        // The e-folding voltage of the curve, *not* the forward drop. Using the forward
        // drop here would make the diode roughly n-order-of-magnitude too stiff to turn on.
        let vscale = model.emission_coefficient * VT;
        let vdcoef = 1.0 / vscale;

        // Critical voltage for limiting: current reaches vscale/sqrt(2) here.
        let vcrit = vscale * (vscale / (std::f64::consts::SQRT_2 * leakage)).ln();
        let vzcrit = VT * (VT / (std::f64::consts::SQRT_2 * leakage)).ln();
        let zoffset = if zvoltage == 0.0 {
            0.0
        } else {
            // Offset that yields 5 mA at the breakdown voltage.
            let i = -0.005;
            zvoltage - (-(1.0 + i / leakage)).ln() / VZCOEF
        };

        // A model with series resistance needs a third node slot; one without needs the
        // slot released again.
        self.base.ensure_slots(model.node_slots());
        self.leakage = leakage;
        self.fwdrop = model.forward_voltage_drop();
        self.zvoltage = zvoltage;
        self.series_resistance = model.series_resistance;
        self.vscale = vscale;
        self.vdcoef = vdcoef;
        self.vcrit = vcrit;
        self.vzcrit = vzcrit;
        self.zoffset = zoffset;
        // The damping logic compares against the previous iteration's voltage, which is no
        // longer meaningful under a different model.
        self.lastvoltdiff = 0.0;
    }

    /// A convenience diode with no series resistance, the CircuitJS `default` model.
    pub fn default_diode() -> Self {
        let model = builtin_model("default").expect("built-in model");
        Self::new(
            Point::new(0, 0),
            Point::new(100, 0),
            0,
            &model,
            DiodeKind::Plain,
        )
    }

    pub fn model(&self) -> DiodeModel {
        DiodeModel {
            saturation_current: self.leakage,
            // vscale is the e-folding voltage, so the coefficient is recoverable from it.
            emission_coefficient: self.vscale / VT,
            breakdown_voltage: self.zvoltage,
            series_resistance: self.series_resistance,
        }
    }

    /// Swap the model in place. Port of `DiodeElm.setup` called from `updateModels`.
    ///
    /// If the new model has a different series resistance the element gains or loses a node,
    /// so the circuit has to be re-analysed afterwards.
    pub fn set_model(&mut self, model: &DiodeModel) {
        self.apply_model(model);
    }

    /// The voltage across the junction, which is *not* the voltage across the element once a
    /// series resistance is present. Port of `volts[0] - volts[diodeEndNode]`.
    fn junction_voltage(&self) -> f64 {
        let end = self.diode_end_node();
        self.base.volts[0] - self.base.volts[end]
    }

    /// Index into `nodes`/`volts` of the junction's cathode side. Port of `diodeEndNode`.
    fn diode_end_node(&self) -> usize {
        if self.series_resistance > 0.0 {
            2
        } else {
            1
        }
    }

    /// Port of `Diode.calculateCurrent`.
    fn calculate_current(&mut self) {
        let v = self.junction_voltage();
        self.base.current = if v >= 0.0 || self.zvoltage == 0.0 {
            self.leakage * ((v * self.vdcoef).exp() - 1.0)
        } else {
            self.leakage * ((v * self.vdcoef).exp() - ((-v - self.zoffset) * VZCOEF).exp() - 1.0)
        };
    }

    /// Damped Newton step. Port of `Diode.limitStep`.
    ///
    /// A pure Newton iteration on a diode jumps by orders of magnitude in current between
    /// iterations and will not converge. If the proposed step is large enough that the
    /// current would change by more than a factor of `e^2`, the step is replaced with the
    /// smallest one that keeps the current consistent with the previous linearised model,
    /// and the iteration is flagged as not yet converged.
    fn limit_step(&self, vnew: f64, vold: f64, ctx: &SimContext) -> f64 {
        let mut vnew = vnew;

        if vnew > self.vcrit && (vnew - vold).abs() > (self.vscale + self.vscale) {
            if vold > 0.0 {
                let arg = 1.0 + (vnew - vold) / self.vscale;
                vnew = if arg > 0.0 {
                    vold + self.vscale * arg.ln()
                } else {
                    self.vcrit
                };
            } else {
                // 1/vscale is the slope of the load line.
                vnew = self.vscale * (vnew / self.vscale).ln();
            }
            ctx.set_converged(false);
        } else if vnew < 0.0 && self.zoffset != 0.0 {
            // Zener breakdown: same logic with the values translated and the steeper
            // Zener-specific scale substituted for the normal one.
            let mut vn = -vnew - self.zoffset;
            let vold = -vold - self.zoffset;

            if vn > self.vzcrit && (vn - vold).abs() > (VT + VT) {
                if vold > 0.0 {
                    let arg = 1.0 + (vn - vold) / VT;
                    vn = if arg > 0.0 {
                        vold + VT * arg.ln()
                    } else {
                        self.vzcrit
                    };
                } else {
                    vn = VT * (vn / VT).ln();
                }
                ctx.set_converged(false);
            }
            vnew = -(vn + self.zoffset);
        }
        vnew
    }
}

impl DiodeModel {
    /// Number of `nodes`/`volts` slots a model needs: two posts, plus a hidden node when the
    /// junction is separated from the cathode by a real resistor.
    pub fn node_slots(&self) -> usize {
        2 + usize::from(self.series_resistance > 0.0)
    }

    /// Builder for the Zener case: the default curve plus a breakdown voltage.
    fn with_breakdown_voltage(mut self, zvoltage: f64) -> Self {
        self.breakdown_voltage = zvoltage;
        self
    }
}

impl_element!(
    Diode,
    posts = 2,
    extra = {
        /// A diode's conductance depends on its bias, so the whole circuit becomes nonlinear
        /// and the engine switches to the Newton loop.
        fn non_linear(&self) -> bool {
            true
        }

        /// The series resistor needs a node of its own, which is what turns a 1N4148 or an LED
        /// into a 3-node element. Port of `DiodeElm.getInternalNodeCount`.
        fn internal_node_count(&self) -> usize {
            usize::from(self.series_resistance > 0.0)
        }

        fn reset_extra(&mut self) {
            self.lastvoltdiff = 0.0;
        }

        /// The left-hand side is rewritten every iteration, so both rows must be protected
        /// from matrix simplification. Port of `DiodeElm.stamp`.
        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            let n0 = self.base.nodes[0];
            let nj = self.base.nodes[self.diode_end_node()];
            stamper.mark_non_linear(n0);
            stamper.mark_non_linear(nj);
            if self.series_resistance > 0.0 {
                // The cathode side of the element is reached through a plain resistor.
                let n1 = self.base.nodes[1];
                stamper.stamp_resistor(n1, nj, self.series_resistance);
            }
        }

        /// Port of `DiodeElm.doStep`.
        fn do_step(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            let n0 = self.base.nodes[0];
            let nj = self.base.nodes[self.diode_end_node()];
            let raw = self.junction_voltage();

            if (raw - self.lastvoltdiff).abs() > 0.01 {
                ctx.set_converged(false);
            }
            let voltdiff = self.limit_step(raw, self.lastvoltdiff, ctx);
            self.lastvoltdiff = voltdiff;

            // A tiny conductance across each junction prevents a singular matrix.
            let mut gmin = self.leakage * 0.01;
            let sub = ctx.sub_iterations.get();
            if sub > 100 {
                // Struggling to converge: ramp the parallel conductance up, capped at 0.1 S.
                gmin = (-9.0 * 10f64.ln() * (1.0 - sub as f64 / 3000.0)).exp();
                if gmin > 0.1 {
                    gmin = 0.1;
                }
            }

            if voltdiff >= 0.0 || self.zvoltage == 0.0 {
                // Regular diode, or a forward-biased Zener.
                let eval = (voltdiff * self.vdcoef).exp();
                let geq = self.vdcoef * self.leakage * eval + gmin;
                let nc = (eval - 1.0) * self.leakage - geq * voltdiff;
                stamper.stamp_conductance(n0, nj, geq);
                stamper.stamp_current_source(n0, nj, nc);
            } else {
                // Reverse-biased Zener: approximate the breakdown curve with an exponential
                // similar to the ideal Shockley curve.
                let eval = (voltdiff * self.vdcoef).exp();
                let ezl = ((-voltdiff - self.zoffset) * VZCOEF).exp();
                let geq = self.leakage * (self.vdcoef * eval + VZCOEF * ezl) + gmin;
                let nc = self.leakage * (eval - ezl - 1.0) + geq * (-voltdiff);
                stamper.stamp_conductance(n0, nj, geq);
                stamper.stamp_current_source(n0, nj, nc);
            }
        }

        /// Port of `DiodeElm.stepFinished`: a current this large means the model and the bias
        /// disagree so badly that continuing only produces noise.
        fn step_finished(&mut self, ctx: &SimContext) {
            if self.base.current.abs() > 1e12 {
                ctx.request_stop();
            }
        }
    }
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::Element;

    #[test]
    fn builtin_models_round_trip_their_forward_drops() {
        // The `default` model's whole reason for existing is its ~0.8 V drop.
        let m = builtin_model("default").unwrap();
        assert!((m.forward_voltage_drop() - 0.805904783).abs() < 1e-8);
        // The LED model is what gives an LED its ~2.2 V.
        let led = builtin_model("default-led").unwrap();
        assert!((led.forward_voltage_drop() - 2.229).abs() < 0.01);
        assert!(builtin_model("no-such-model").is_none());
    }

    #[test]
    fn series_resistance_creates_a_third_node() {
        let mut d = Diode::default_diode();
        assert_eq!(Element::internal_node_count(&d), 0);
        assert_eq!(d.base.volts.len(), 2);

        d.set_model(&builtin_model("1N4148").unwrap());
        assert_eq!(Element::internal_node_count(&d), 1);
        assert_eq!(d.base.volts.len(), 3);
        // The junction is now between post 0 and the hidden node, not between the posts.
        assert_eq!(d.diode_end_node(), 2);
    }

    #[test]
    fn zener_clamps_reverse_current_at_the_breakdown_voltage() {
        let model = Diode::zener(Point::new(0, 0), Point::new(100, 0), 0, 5.6).model();
        assert_eq!(model.breakdown_voltage, 5.6);
        // 5 mA is the current the Zener offset is calibrated to pass at exactly Vz.
        let d = Diode::zener(Point::new(0, 0), Point::new(100, 0), 0, 5.6);
        assert!((d.zoffset - (5.6 - (-(1.0 + -0.005 / d.leakage)).ln() / VZCOEF)).abs() < 1e-12);
    }
}
