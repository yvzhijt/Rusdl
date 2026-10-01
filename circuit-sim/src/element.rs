//! The element contract, mirroring the simulation-facing half of `CircuitElm.java`.
//!
//! circuitjs1's `CircuitElm` mixes three concerns: geometry for drawing, UI editing, and
//! the MNA stamping callbacks. Only the third group belongs in a headless engine, so this
//! trait keeps exactly that group. Everything an element needs to *know* about the
//! simulation arrives through [`SimContext`]; everything it needs to *write* goes through
//! [`Stamper`].

use crate::stamper::Stamper;
use std::cell::Cell;

/// Integer grid coordinate. The engine identifies nodes by post position, exactly as
/// circuitjs1 does — that is what makes wire-to-wire and post-to-post coincidence work.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Point {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// Read-only view of the simulation state that elements may consult during stamping.
///
/// `converged` is a [`Cell`] because elements clear it from `doStep` while only holding a
/// shared reference. This is the same mutable flag circuitjs1 keeps on `CirSim`.
pub struct SimContext {
    /// Current timestep in seconds.
    pub time_step: f64,
    /// Simulation time in seconds, at the start of the current iteration.
    pub time: f64,
    /// True while hunting for the DC operating point.
    pub dc_analysis: bool,
    /// Number of circuit nodes **including** ground. Voltage-source row indices are
    /// computed as `node_count + vs`, matching the Java original's 1-based convention.
    pub node_count: usize,
    /// Index of the current Newton sub-iteration. Elements use this to progressively add
    /// conductance when they fail to converge (`Diode`'s `gmin` ramp).
    pub sub_iterations: Cell<usize>,
    converged: Cell<bool>,
    stop_requested: Cell<bool>,
}

impl SimContext {
    pub fn new(time_step: f64, time: f64, dc_analysis: bool, node_count: usize) -> Self {
        Self {
            time_step,
            time,
            dc_analysis,
            node_count,
            sub_iterations: Cell::new(0),
            converged: Cell::new(true),
            stop_requested: Cell::new(false),
        }
    }

    /// Whether every nonlinear element accepted the previous solution. Elements clear
    /// this from `doStep` when their operating point moved too far.
    pub fn converged(&self) -> bool {
        self.converged.get()
    }

    pub fn set_converged(&self, v: bool) {
        self.converged.set(v);
    }

    /// Ask the engine to abandon the run after this timestep.
    ///
    /// The Java original calls `sim.stop(...)` from `stepFinished`, which is only
    /// reachable there because a broken operating point keeps producing nonsense. A
    /// [`Cell`] mirrors how `converged` is threaded through a shared reference.
    pub fn request_stop(&self) {
        self.stop_requested.set(true);
    }

    /// Whether any element asked to abandon the run during this timestep.
    pub fn stop_requested(&self) -> bool {
        self.stop_requested.get()
    }
}

/// A circuit element.
///
/// Implementors keep their own `nodes` and `volts` arrays; the engine fills them in via
/// [`Element::set_node`] and [`Element::set_node_voltage`] respectively. All indices
/// handed to and returned by this trait are 1-based node numbers, where 0 is ground.
pub trait Element {
    // --- topology, queried once during analysis ---------------------------

    /// Number of user-visible connection posts.
    fn post_count(&self) -> usize {
        2
    }

    /// Number of hidden nodes the implementation needs (e.g. a transistor's internal
    /// collector node).
    fn internal_node_count(&self) -> usize {
        0
    }

    /// Number of extra MNA rows this element needs for its voltage sources.
    fn voltage_source_count(&self) -> usize {
        0
    }

    /// True if the element forces the Newton loop (its stamp depends on the solution).
    fn non_linear(&self) -> bool {
        false
    }

    /// Position of post `n`. The engine keys nodes off these.
    fn post(&self, n: usize) -> Point;

    /// Location of post `n` for `n >= 2`.
    ///
    /// The default returns the second post, which is right for the two-post elements that
    /// dominate. An element whose third post is *derived* from the geometry — a
    /// potentiometer's wiper, an analog switch's control terminal — overrides this.
    fn extra_post(&self, n: usize) -> Point {
        let _ = n;
        self.post(1)
    }

    /// Which post sits at point `p`, or 0 if none does.
    ///
    /// Port of `CircuitElm.getNodeAtPoint`. The two-post case is special-cased in the Java
    /// original because an element's posts may be a sub-grid of what it exposes; returning
    /// "the other post" is correct for the two-terminal elements that dominate here.
    /// The node number of the post sitting on `p`, or 0 if this element has no post there.
    ///
    /// Node 0 is indistinguishable from "not found" here, which is why
    /// [`crate::Circuit::node_at_point`] exists: it reports `None` instead.
    fn node_at_point(&self, p: Point) -> usize {
        (0..self.post_count())
            .find(|&k| self.post(k) == p)
            .map_or(0, |k| self.node(k))
    }

    // --- connectivity predicates, used by validation ----------------------

    /// Number of nodes considered when deciding what is connected to ground.
    fn connection_node_count(&self) -> usize {
        self.post_count()
    }

    /// The `n`th node considered for ground connectivity.
    fn connection_node(&self, n: usize) -> usize {
        self.node(n)
    }

    /// True if nodes `n1` and `n2` are connected through this element, for the purpose of
    /// finding loops and unconnected nodes.
    ///
    /// True for every two-terminal element, which is the base `CircuitElm.getConnection`
    /// default. It only goes false for elements that deliberately do not conduct between
    /// their own posts, such as a gate or a probe, none of which are ported yet.
    fn get_connection(&self, _n1: usize, _n2: usize) -> bool {
        true
    }

    /// True if node `n1` reaches ground through this element regardless of the rest of
    /// the circuit. Only `GroundElm` and `RailElm` override this.
    fn has_ground_connection(&self, _n1: usize) -> bool {
        false
    }

    /// True if this element is a wire (or behaves like one: it ties its posts together
    /// without contributing a matrix row).
    fn is_wire(&self) -> bool {
        false
    }

    /// True if this element ties its posts together like a wire *without* being a wire.
    ///
    /// Used wherever the Java original asks `isWire()` while walking a path or hunting a
    /// loop: the voltage-loop check and `FindPathInfo.findPath`. A closed switch answers
    /// true here — it shorts its posts — while still being a real element with a matrix row,
    /// which is why [`Element::is_wire`] stays false for it.
    fn is_wire_like(&self) -> bool {
        self.is_wire()
    }

    // --- kind predicates, replacing Java `instanceof` checks ---------------

    fn is_ground(&self) -> bool {
        false
    }
    fn is_rail(&self) -> bool {
        false
    }
    fn is_voltage_source(&self) -> bool {
        false
    }
    fn is_current_source(&self) -> bool {
        false
    }
    fn is_capacitor(&self) -> bool {
        false
    }
    fn is_inductor(&self) -> bool {
        false
    }

    // --- engine callbacks --------------------------------------------------

    /// This element as [`Any`](std::any::Any), so callers holding a `Box<dyn Element>` can
    /// downcast to the concrete type.
    fn as_any(&self) -> &dyn std::any::Any;

    /// The mutable counterpart of [`Element::as_any`], for callers that need to drive the
    /// element (toggling a switch, say) rather than just read it.
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    /// Notify the element that its `p`th node is `n`.
    fn set_node(&mut self, p: usize, n: usize);

    /// The global number of the element's `n`th node.
    fn node(&self, n: usize) -> usize;

    /// Notify the element that its `n`th voltage source is row `v`.
    fn set_voltage_source(&mut self, n: usize, v: usize);

    /// Set the voltage of the `n`th node. Implementors must also recompute their
    /// current, matching `setNodeVoltage` calling `calculateCurrent`.
    fn set_node_voltage(&mut self, n: usize, v: f64);

    /// Set the current through the `n`th voltage source, as solved by the engine.
    fn set_current(&mut self, n: usize, c: f64);

    /// Hook to adjust the sign or scale of a solved current before it is stored.
    ///
    /// The default stores the solved value unchanged, which is right for every element that
    /// reports current flowing from post 0 to post 1. `Ground` overrides it because the
    /// solved variable there is the current *out of* the 0V source, i.e. the opposite sense.
    fn adjust_solved_current(&self, c: f64) -> f64 {
        c
    }

    /// Current through the element, for display and `getCurrentIntoNode`.
    fn current(&self) -> f64;

    /// Current at each post, positive when it flows *into* the element through that post.
    ///
    /// A two-terminal element's is just `current()` negated at post 0, which is the
    /// "current entering here" sense the current animation draws markers in. The BJT
    /// overrides this to report its base, collector and emitter currents separately: they
    /// are three different numbers satisfying `Ie = -(Ib + Ic)`, and collapsing them into
    /// the single collector current the solver reports would hide exactly the thing the
    /// animation exists to show.
    fn post_currents(&self) -> Vec<f64> {
        let current = self.current();
        (0..self.post_count())
            .map(|n| if n == 0 { -current } else { current })
            .collect()
    }

    /// Current flowing *into* node `n` through this element.
    ///
    /// Used to reconstruct wire currents, since a wire has the same node at both ends and
    /// so cannot compute its own current from node voltages. Port of
    /// `CircuitElm.getCurrentIntoNode`.
    ///
    /// The argument is a *post* index, not a node number: `CircuitElm.getCurrentIntoNode`
    /// compares it against zero because in practice it is handed the ground node, and
    /// passing the post index reproduces that only where the two coincide. Callers here all
    /// pass the post that shares the point, which makes the comparison mean "post 0" for
    /// every element, including the three-terminal ones that override this.
    fn current_into_node(&self, n: usize) -> f64 {
        // The `post_count() == 2` special case is what makes rails report correctly.
        if n == 0 && self.post_count() == 2 {
            -self.current()
        } else {
            self.current()
        }
    }

    /// Reset all state (voltages, currents, companion models) to the power-on state.
    fn reset(&mut self);

    /// Hook for element state that lives outside the shared geometry/voltage arrays and must
    /// be cleared by [`Element::reset`]. Empty by default.
    ///
    /// This is where a capacitor drops its stored charge or a current source clears its
    /// `broken` flag; the shared `Base` state is already handled for you.
    fn reset_extra(&mut self) {}

    /// Called once per timestep, before the Newton loop, to refresh companion models.
    fn start_iteration(&mut self, _ctx: &SimContext) {}

    /// Stamp the parts of the system that do not change between Newton iterations.
    fn stamp(&mut self, stamper: &mut Stamper, ctx: &SimContext);

    /// Stamp the parts that depend on the current solution estimate.
    ///
    /// Only elements with a companion model or a dynamic source do anything here; the rest
    /// are linear and already fully stamped by [`Element::stamp`].
    fn do_step(&mut self, _stamper: &mut Stamper, _ctx: &SimContext) {}

    /// Called after a timestep completes successfully.
    fn step_finished(&mut self, ctx: &SimContext) {
        let _ = ctx;
    }

    /// Told by the engine that a capacitor was detected as shorted, so it can drop its
    /// stored voltage instead of ringing forever.
    fn shorted(&mut self) {
        self.reset();
    }

    /// Marked by the engine when a current source has no path to ground, so it can be
    /// replaced with a large resistor instead of stamping a singular source.
    fn set_broken(&mut self, _broken: bool) {}
}
