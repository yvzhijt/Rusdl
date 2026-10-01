//! NPN bipolar junction transistor.
//!
//! The SPICE Gummel-Poon transport-factor model, with the two junction capacitances left
//! out. The two junctions are not independent, which is what makes a transistor a
//! nonlinear *multi-terminal* element rather than "two diodes plus a resistor":
//!
//! ```text
//! Vbe = V(b) - V(e)      Vbc = V(b) - V(c)
//! Ib  =  Is/Bf * (e^{Vbe/vt} - 1)  +  Is/Br * (e^{Vbc/vt} - 1)
//! Ic  =  Is * (e^{Vbe/vt} - e^{Vbc/vt})  -  Is/Br * (e^{Vbc/vt} - 1)
//! Ie  = -(Ib + Ic)
//! ```
//!
//! Forward active (`Vbe` well above the knee, `Vbc` well below it) collapses to
//! `Ic = Is * e^{Vbe/vt}` and `Ib = Is/Bf * e^{Vbe/vt}`, so the common-emitter gain is
//! `Bf` — which is the property a biasing exercise is actually about.
//!
//! # Stamping
//!
//! The three terminal currents are functions of the two independent junction voltages, so
//! the device's contribution to each node's KCL row is linearised about the current
//! solution. Because the model is *not* reciprocal, the resulting Jacobian is asymmetric,
//! which is why this element cannot be expressed as a sum of two `stamp_conductance` calls
//! the way the diode is: the collector row's dependence on `Vbc` is not the negative of the
//! base row's. The full 3x3 Jacobian of the current *leaving* each node is therefore
//! stamped entry by entry, and the emitter row is the exact negation of the other two
//! (which is the same statement as `Ie = -(Ib + Ic)`).

use crate::element::{Point, SimContext};
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// Electron thermal voltage at SPICE's default 27 C, matching [`crate::elements::Diode`].
const VT: f64 = 0.025865;
/// Saturation current, the SPICE default.
const IS: f64 = 1e-14;
/// Reverse common-base current gain, the SPICE default.
const BR: f64 = 1.0;

/// The junction voltage at which the exponential has to be damped rather than evaluated:
/// the current reaches `vt / sqrt(2)` there, and beyond it a Newton step can overshoot by
/// orders of magnitude in a single iteration.
fn vcrit() -> f64 {
    VT * (VT / (std::f64::consts::SQRT_2 * IS)).ln()
}

/// Index of the three terminals within `nodes`/`volts`.
const BASE: usize = 0;
const COLLECTOR: usize = 1;
const EMITTER: usize = 2;

/// An NPN bipolar junction transistor: a base, a collector, and an emitter.
///
/// The posts are those three terminals, in that order. A PNP would be the same model with
/// every junction voltage negated; it is not implemented, so a user who wants one has to
/// mirror the element and swap the supply rails.
#[derive(Clone, Debug)]
pub struct Bjt {
    base: Base,
    /// Emitter position, held separately because [`Base`] only stores two points.
    emitter: Point,
    /// Forward common-emitter current gain.
    beta: f64,
    saturation_current: f64,
    reverse_beta: f64,
    /// Damped junction voltages from the previous `do_step`, for the step limiter.
    last_vbe: f64,
    last_vbc: f64,
    /// Base and emitter currents, alongside the collector current in `Base::current`.
    ///
    /// The solver only needs the collector current, so these two are kept purely for the
    /// current animation, which has to march three different phases to draw the device.
    base_current: f64,
    emitter_current: f64,
}

impl Bjt {
    /// An NPN transistor with its base at `base`, collector at `collector` and emitter at
    /// `emitter`.
    pub fn new(base: Point, collector: Point, emitter: Point, flags: i32, beta: f64) -> Self {
        Self {
            base: Base::from_points(base, collector, flags, 3),
            emitter,
            beta: sanitise_beta(beta),
            saturation_current: IS,
            reverse_beta: BR,
            last_vbe: 0.0,
            last_vbc: 0.0,
            base_current: 0.0,
            emitter_current: 0.0,
        }
    }

    pub fn from_base(base: Base, emitter: Point, beta: f64) -> Self {
        Self {
            base,
            emitter,
            beta: sanitise_beta(beta),
            saturation_current: IS,
            reverse_beta: BR,
            last_vbe: 0.0,
            last_vbc: 0.0,
            base_current: 0.0,
            emitter_current: 0.0,
        }
    }

    /// Forward common-emitter current gain, `beta`.
    pub fn beta(&self) -> f64 {
        self.beta
    }

    /// A non-positive or non-finite gain would collapse the base conductance to zero and
    /// leave the base node floating, so it is clamped to something usable instead.
    pub fn set_beta(&mut self, beta: f64) {
        self.beta = sanitise_beta(beta);
    }

    /// Base-emitter junction voltage, `V(b) - V(e)`.
    pub fn vbe(&self) -> f64 {
        self.base.volts[BASE] - self.base.volts[EMITTER]
    }

    /// Base-collector junction voltage, `V(b) - V(c)`.
    pub fn vbc(&self) -> f64 {
        self.base.volts[BASE] - self.base.volts[COLLECTOR]
    }

    /// The three terminal currents, in post order, each positive flowing *into* the device.
    ///
    /// An active NPN draws base and collector current in and pushes emitter current out, so
    /// the emitter entry comes out negative and the three still sum to zero.
    pub fn post_currents(&self) -> Vec<f64> {
        vec![self.base_current, self.base.current, self.emitter_current]
    }

    /// The emitter is the third post, which the element contract only reaches through
    /// `extra_post`.
    fn calculate_current(&mut self) {}
}

fn sanitise_beta(beta: f64) -> f64 {
    if beta.is_finite() && beta > 0.0 {
        beta
    } else {
        100.0
    }
}

/// Replaces one Newton step with the largest one that keeps the junction current
/// consistent with the previous linearisation.
///
/// A pure Newton iteration on an exponential jumps by orders of magnitude per step and
/// will not converge from a cold start. If the proposed step is far enough that the
/// current would change by more than a factor of `e^2`, it is pulled back to the value
/// that keeps `dv` and `dI` proportional, and the iteration is flagged as not yet
/// converged. This is the same limiter the diode uses, with the same `vcrit`/`vt` scaling.
fn limit_junction(vnew: f64, vold: f64, vcrit: f64, ctx: &SimContext) -> f64 {
    let mut vnew = vnew;
    if vnew > vcrit && (vnew - vold).abs() > (VT + VT) {
        if vold > 0.0 {
            let arg = 1.0 + (vnew - vold) / VT;
            vnew = if arg > 0.0 {
                vold + VT * arg.ln()
            } else {
                vcrit
            };
        } else {
            // 1/VT is the slope of the load line out of the origin.
            vnew = VT * (vnew / VT).ln();
        }
        ctx.set_converged(false);
    }
    vnew
}

impl_element!(
    Bjt,
    posts = 3,
    extra = {
        fn extra_post(&self, _n: usize) -> Point {
            self.emitter
        }

        /// The conductances depend on the bias, so the whole circuit becomes nonlinear and
        /// the engine switches to the Newton loop.
        fn non_linear(&self) -> bool {
            true
        }

        fn reset_extra(&mut self) {
            self.last_vbe = 0.0;
            self.last_vbc = 0.0;
            self.base_current = 0.0;
            self.emitter_current = 0.0;
        }

        /// Reports base, collector and emitter separately so the current animation can draw
        /// three phases instead of one.
        fn post_currents(&self) -> Vec<f64> {
            Bjt::post_currents(self)
        }

        /// Every KCL row this element touches is rewritten in `do_step`, so all three are
        /// protected from matrix simplification — otherwise a node held at a constant
        /// voltage would be folded away with a Jacobian that is about to change.
        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            for node in 0..3 {
                stamper.mark_non_linear(self.base.nodes[node]);
            }
        }

        fn do_step(&mut self, stamper: &mut Stamper, ctx: &SimContext) {
            // Everything below is indexed in *junction* order — base, emitter, collector —
            // because that is the order the two junction voltages are naturally written in.
            // It is not the post order, so the two are kept as separate triples and the
            // emitter's row is the third one rather than the second.
            let (nb, nc, ne) = (
                self.base.nodes[BASE],
                self.base.nodes[COLLECTOR],
                self.base.nodes[EMITTER],
            );
            // Rows are the two junctions' own terminals (base, collector); columns are in
            // junction order (base, emitter, collector) because that is the order the two
            // junction voltages are written in. The emitter's own row is assembled
            // afterwards. Keeping the two orders spelled out is what stops them being
            // confused — they are not the same, and mixing them up silently stamps the
            // collector's equation at the emitter and leaves the collector floating.
            let row_nodes = [nb, nc];
            let col_nodes = [nb, ne, nc];
            let volts = [
                self.base.volts[BASE],
                self.base.volts[EMITTER],
                self.base.volts[COLLECTOR],
            ];

            let raw = [self.vbe(), self.vbc()];

            // The base current flowing in from the bias network and the collector current
            // flowing out to the load are the two quantities a solve has to agree on, so a
            // junction that has moved appreciably means the iteration is not there yet.
            // Without this the engine would happily accept the first result it manages to
            // produce, because a junction biased *below* the knee never trips the limiter
            // above and so never reports non-convergence.
            //
            // The threshold is in volts across a junction whose exponential has a scale of
            // `vt`, so 2 mV is about 8% of current — the same relative accuracy the diode's
            // 10 mV threshold gives at twice the thermal voltage.
            if (raw[0] - self.last_vbe).abs() > 0.002 || (raw[1] - self.last_vbc).abs() > 0.01 {
                ctx.set_converged(false);
            }

            let vcrit = vcrit();
            let vbe = limit_junction(raw[0], self.last_vbe, vcrit, ctx);
            let vbc = limit_junction(raw[1], self.last_vbc, vcrit, ctx);
            self.last_vbe = vbe;
            self.last_vbc = vbc;

            // The exponent is capped well below overflow. The limiter above already keeps
            // the argument near `vcrit`, but a caller can hand the engine a nonsensical
            // bias, and a NaN here would poison the whole matrix.
            let ef = (vbe / VT).min(80.0).exp();
            let er = (vbc / VT).min(80.0).exp();

            let is = self.saturation_current;
            let ib = (is / self.beta) * (ef - 1.0) + (is / self.reverse_beta) * (er - 1.0);
            let ic = is * (ef - er) - (is / self.reverse_beta) * (er - 1.0);

            // dI/dV of each junction current, per volt.
            let gbf = is / self.beta * ef / VT;
            let gbr = is / self.reverse_beta * er / VT;
            let gcf = is * ef / VT;
            let gcr = is * er / VT + gbr;

            // A tiny conductance across each junction keeps the matrix solvable while the
            // device is cut off, when the exponential conductances are numerically zero.
            // Ramped up if the iteration is struggling, exactly as the diode's `gmin` is.
            let mut gmin = is * 0.01;
            let sub = ctx.sub_iterations.get();
            if sub > 100 {
                gmin = ((-9.0 * 10f64.ln()) * (1.0 - sub as f64 / 3000.0)).exp();
                if gmin > 0.1 {
                    gmin = 0.1;
                }
            }

            // Currents *leaving* each node through the device, and their Jacobian with
            // respect to (V(base), V(emitter), V(collector)). The base and collector rows
            // are just `Ib` and `Ic`; the emitter row is the exact negation of their sum,
            // which is `Ie = -(Ib + Ic)` restated. The Jacobian is deliberately *not*
            // symmetric, which is exactly what a non-reciprocal device looks like and why
            // this cannot be written as a sum of two `stamp_conductance` calls.
            let jac = [[gbf + gbr, -gbf, -gbr], [gcf - gcr, -gcf, gcr]];
            let currents = [ib, ic];

            // The linear model is anchored on the *limited* junction voltages, but the
            // right-hand side has to reproduce the model at the *node* voltages the last
            // solve produced, because that is what every other row was written against.
            // `anchor` is the node-voltage triple consistent with `vbe`/`vbc`; evaluating
            // the Jacobian there turns the offset into the same `I - g * v` form the diode
            // uses. This correction is the whole reason the limiter works: anchored at the
            // limited voltage instead, the stamped model would be evaluated at the wrong
            // point and the iteration would oscillate between a huge overestimate and
            // cutoff rather than ramping up to the knee.
            let anchor = [volts[1] + vbe, volts[1], volts[1] + vbe - vbc];

            let mut rhs = [0.0; 2];
            for row in 0..2 {
                for (col, node) in col_nodes.iter().enumerate() {
                    stamper.stamp_matrix(row_nodes[row], *node, jac[row][col]);
                }
                rhs[row] =
                    (0..3).map(|col| jac[row][col] * anchor[col]).sum::<f64>() - currents[row];
                stamper.stamp_right_side(row_nodes[row], rhs[row]);
            }
            // Emitter row: the negated sum, so its right-hand side is the negated sum too.
            for (col, node) in col_nodes.iter().enumerate() {
                let j = -(jac[0][col] + jac[1][col]);
                stamper.stamp_matrix(ne, *node, j);
            }
            stamper.stamp_right_side(ne, -(rhs[0] + rhs[1]));

            // The `gmin` conductances are pure resistances, so they contribute nothing to
            // any right-hand side and can be stamped on their own afterwards.
            stamper.stamp_conductance(nb, ne, gmin);
            stamper.stamp_conductance(nb, nc, gmin);

            // The reported current is the collector current: it is the one a biasing or
            // gain exercise asks for, and it is what the desktop editor animates. The base
            // and emitter currents ride along for the animation, which marches a separate
            // phase down each of the three leads.
            self.base.current = ic;
            self.base_current = ib;
            // The emitter row is the exact negation of the other two, so this is `Ie` in the
            // same "into the device" sense: negative for an active NPN.
            self.emitter_current = -(ib + ic);
        }

        /// A current this large means the model and the bias disagree badly enough that
        /// continuing only produces noise.
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
    use crate::elements::{Resistor, VoltageSource};
    use crate::{Circuit, Element};

    /// Node layout shared by the biasing tests: everything in grid units, `vcc` at the top,
    /// the base fed through `rb`, the collector loaded through `rc`, the emitter grounded.
    fn common_emitter(vcc: f64, rb: f64, rc: f64, beta: f64) -> Circuit {
        let rail = Point::new(0, 400);
        let collector = Point::new(0, 300);
        let base = Point::new(0, 100);
        let emitter = Point::new(0, 0);

        let mut c = Circuit::new();
        c.add(Box::new(VoltageSource::dc_between(emitter, rail, vcc)));
        c.add(Box::new(Resistor::between(rail, base, rb)));
        c.add(Box::new(Resistor::between(rail, collector, rc)));
        c.add(Box::new(Bjt::new(base, collector, emitter, 0, beta)));
        c
    }

    fn transistor_of(circuit: &Circuit) -> &Bjt {
        circuit.elements()[3]
            .as_any()
            .downcast_ref::<Bjt>()
            .expect("transistor")
    }

    /// The whole point of the model: a base bias sets the collector current, and the
    /// common-emitter gain is `beta`.
    ///
    /// The numbers are pinned tightly because they are the ones that expose a sign or index
    /// slip: 10 V through 1 M into the base settles at `Vt * ln(beta * I / Is)`, and the
    /// collector then sits a `beta` times the base current below the rail.
    #[test]
    fn a_base_bias_sets_the_collector_current() {
        let mut c = common_emitter(10.0, 1e6, 1_000.0, 100.0);
        c.analyze().expect("should analyse");
        // The first timestep spends most of its sub-iterations ramping the base up to the
        // knee from a cold start, so a few steps are needed before the operating point has
        // stopped moving.
        for _ in 0..5 {
            c.step().expect("should step");
        }

        let t = transistor_of(&c);
        let ib = (10.0 - t.vbe()) / 1e6;
        // Is*(e^{Vbe/vt} - 1)/beta == Ib, solved for Vbe.
        let expected_vbe = VT * ((ib * 100.0 + IS) / IS).ln();
        assert!((t.vbe() - expected_vbe).abs() < 1e-3, "vbe = {}", t.vbe());
        // The collector junction is reverse biased, so this really is forward active.
        assert!(t.vbc() < -8.0, "vbc = {}", t.vbc());
        assert!(
            (t.current() - 100.0 * ib).abs() < 1e-6,
            "ic = {} A, expected {} A",
            t.current(),
            100.0 * ib
        );
        // And the collector node reflects the load: 10 V minus Ic*Rc.
        let nc = c.node_at_point(Point::new(0, 300)).expect("collector node");
        assert!(
            (c.node_voltage(nc) - (10.0 - t.current() * 1_000.0)).abs() < 1e-3,
            "vc = {}",
            c.node_voltage(nc)
        );
    }

    /// With the base held at the emitter's potential both junctions are unbiased and the
    /// device is in cutoff, so essentially no collector current flows.
    #[test]
    fn an_undriven_base_cuts_the_device_off() {
        let rail = Point::new(0, 400);
        let collector = Point::new(0, 300);
        let emitter = Point::new(0, 0);

        let mut c = Circuit::new();
        c.add(Box::new(VoltageSource::dc_between(emitter, rail, 10.0)));
        c.add(Box::new(Resistor::between(rail, collector, 1_000.0)));
        // Base tied straight to the emitter, so Vbe = 0.
        c.add(Box::new(Resistor::between(emitter, emitter, 1.0)));
        c.add(Box::new(Bjt::new(emitter, collector, emitter, 0, 100.0)));
        c.analyze().expect("should analyse");
        c.step().expect("should step");

        let t = c.elements()[3].as_any().downcast_ref::<Bjt>().expect("bjt");
        assert!(t.vbe().abs() < 1e-9);
        assert!(t.current().abs() < 1e-9, "cutoff ic = {}", t.current());
    }

    /// The three terminals must stay distinct, and the emitter is the third post rather
    /// than the second.
    #[test]
    fn the_emitter_is_the_third_post() {
        let base = Point::new(0, 0);
        let collector = Point::new(100, 0);
        let emitter = Point::new(0, 100);
        let t = Bjt::new(base, collector, emitter, 0, 100.0);
        assert_eq!(Element::post_count(&t), 3);
        assert_eq!(Element::post(&t, BASE), base);
        assert_eq!(Element::post(&t, COLLECTOR), collector);
        assert_eq!(Element::post(&t, EMITTER), emitter);
    }
}
