//! The simulation engine.
//!
//! Port of the solver half of `CirSim.java`: node analysis (lines 1697-1972), circuit
//! analysis and stamping (2060-2280), the stamping helpers (2423-2546), and the timestep
//! loop (2573-2798).
//!
//! # Pipeline
//!
//! 1. [`Circuit::analyze`] — merge wire endpoints, pick the ground node, allocate node
//!    numbers, and validate that the circuit is solvable. This is structural and only has
//!    to run when the topology changes.
//! 2. [`Circuit::stamp_circuit`] — build the MNA system, simplify it, and (for linear
//!    circuits) factor it once.
//! 3. [`Circuit::step`] — advance one timestep: refresh companion models, run the Newton
//!    loop until convergence, and publish the solution back to the elements.
//!
//! # Why wires are not in the matrix
//!
//! circuitjs1 removes wires from the matrix entirely and merges their endpoints during node
//! allocation. A wire then has a single node at both ends, so it cannot compute a current
//! from node voltages; instead the current is reconstructed from the neighbouring
//! elements' currents in [`Circuit::calc_wire_currents`]. This keeps the matrix small
//! (each wire would otherwise cost two rows) and non-singular, and it is the single most
//! important structural decision to carry over faithfully.

use crate::element::{Element, Point, SimContext};
use crate::stamper::{RowType, Stamper, StamperError};
use std::collections::{HashMap, HashSet};

/// A node and the element terminals attached to it.
#[derive(Clone, Debug, Default)]
struct CircuitNode {
    /// `(element index, terminal number within that element)` pairs.
    links: Vec<(usize, usize)>,
    /// True for nodes the element invented (a transistor's internal node) rather than ones
    /// a user can see. Hidden nodes are never auto-grounded.
    internal: bool,
}

/// Everything needed to reconstruct one wire's current.
#[derive(Clone, Debug)]
struct WireInfo {
    wire: usize,
    /// The wire terminal used as the reference point.
    post: usize,
    /// Elements sharing the opposite terminal, whose currents sum to the wire current.
    neighbors: Vec<(usize, usize)>,
}

/// Which connectivity rule a `FindPathInfo` search obeys. Port of the `FindPathInfo`
/// `type` constants.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PathType {
    /// A path that avoids current sources: used for inductors and current sources, which
    /// would otherwise block any current.
    Induct,
    /// A path made only of voltage sources, wires and ground: used to detect loops that
    /// have no resistance.
    Voltage,
    /// A path made only of wires: used to detect a shorted capacitor.
    Short,
    /// A path made only of capacitors: used to detect a cap loop with no resistance.
    CapV,
}

/// Why the engine refused to run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SimError {
    /// The matrix had a live row with nothing left to determine its unknown.
    MatrixError,
    /// The matrix was structurally singular.
    SingularMatrix,
    /// Two voltage sources (or wire-like elements) form a loop with no resistance.
    VoltageLoop,
    /// An inductor or current source has no path that avoids current sources.
    NoCurrentPath,
    /// A short across a capacitor.
    ShortedCapacitor,
    /// A capacitor loop with no resistance.
    CapacitorLoop,
    /// A path from a supply rail to ground with no resistance.
    RailToGround,
    /// The matrix picked up a NaN or infinity.
    NonFinite,
    /// The Newton loop hit its iteration limit.
    ConvergenceFailed,
    /// A wire loop prevents the wire-current reconstruction from terminating.
    WireLoop,
    /// An element reported its operating point was physically absurd.
    MaxCurrentExceeded,
}

impl std::fmt::Display for SimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Self::MatrixError => "Matrix error",
            Self::SingularMatrix => "Singular matrix!",
            Self::VoltageLoop => "Voltage source/wire loop with no resistance!",
            Self::NoCurrentPath => "Inductor or current source with no current path",
            Self::ShortedCapacitor => "Capacitor shorted",
            Self::CapacitorLoop => "Capacitor loop with no resistance!",
            Self::RailToGround => "Path to ground with no resistance!",
            Self::NonFinite => "nan/infinite matrix!",
            Self::ConvergenceFailed => "Convergence failed!",
            Self::WireLoop => "wire loop detected",
            Self::MaxCurrentExceeded => "max current exceeded",
        };
        f.write_str(s)
    }
}

impl std::error::Error for SimError {}

/// The circuit simulator.
pub struct Circuit {
    elements: Vec<Box<dyn Element>>,
    node_list: Vec<CircuitNode>,
    /// Maps a post position to the wire-closure group it belongs to.
    ///
    /// circuitjs1 achieves this by storing the *same* `NodeMapEntry` object under both of a
    /// wire's endpoints, so mutating the entry in `makeNodeList` assigns one node to both.
    /// Rust has no reference identity to lean on here, so the group is an explicit id and
    /// [`Circuit::group_node`] holds its allocation.
    wire_groups: HashMap<Point, usize>,
    /// Node number allocated to each wire-closure group, or `None` while unallocated.
    ///
    /// Allocation is deliberately deferred until `makeNodeList` so node numbering matches
    /// the Java original: it walks elements in order and only then decides numbers, and
    /// changing that order changes which node gets auto-grounded, which changes results.
    group_node: Vec<Option<usize>>,
    post_count_map: HashMap<Point, usize>,
    wire_info_list: Vec<WireInfo>,
    unconnected_nodes: Vec<usize>,
    /// For each voltage-source row, the index of the element that owns it.
    voltage_sources: Vec<usize>,

    stamper: Stamper,
    node_voltages: Vec<f64>,
    last_node_voltages: Vec<f64>,
    ctx: SimContext,

    /// Current simulation time, seconds.
    pub time: f64,
    /// Current timestep, seconds.
    pub time_step: f64,
    /// Largest timestep the engine will use, seconds.
    pub max_time_step: f64,
    /// Smallest timestep before giving up on convergence, seconds.
    pub min_time_step: f64,
    /// Whether the engine may halve/double the timestep to resolve convergence.
    pub adjust_time_step: bool,
    /// True while hunting for the DC operating point.
    pub dc_analysis: bool,
    /// True if any element forces the Newton loop.
    pub circuit_non_linear: bool,
    /// Number of timesteps completed.
    pub steps: usize,
    /// Sub-iterations used by the most recent timestep.
    pub sub_iterations: usize,
    /// Why the engine stopped, if it did.
    pub error: Option<SimError>,
    /// Index of the element that triggered `error`, if any.
    pub error_element: Option<usize>,
    /// Whether a valid system has been stamped and the circuit is ready to run.
    analysed: bool,
    /// Consecutive easy timesteps, which earn the right to double `time_step`. Seeded high
    /// so the first steps start at `max_time_step`. Port of `CirSim.goodIterations`.
    good_iterations: u32,
    /// Whether the last timestep converged on its first Newton iterations. Port of
    /// `CirSim.goodIteration`.
    good_iteration: bool,
}

impl Default for Circuit {
    fn default() -> Self {
        Self::new()
    }
}

impl Circuit {
    /// Creates an empty circuit with circuitjs1's default timing constants.
    pub fn new() -> Self {
        let max_time_step = 5e-6;
        Self {
            elements: Vec::new(),
            node_list: Vec::new(),
            wire_groups: HashMap::new(),
            group_node: Vec::new(),
            post_count_map: HashMap::new(),
            wire_info_list: Vec::new(),
            unconnected_nodes: Vec::new(),
            voltage_sources: Vec::new(),
            stamper: Stamper::new(1, 0),
            node_voltages: Vec::new(),
            last_node_voltages: Vec::new(),
            ctx: SimContext::new(max_time_step, 0.0, false, 1),
            time: 0.0,
            time_step: max_time_step,
            max_time_step,
            min_time_step: 1e-11,
            adjust_time_step: true,
            dc_analysis: false,
            circuit_non_linear: false,
            steps: 0,
            sub_iterations: 0,
            error: None,
            error_element: None,
            analysed: false,
            good_iterations: 100,
            good_iteration: true,
        }
    }

    // --- construction -----------------------------------------------------

    /// Appends an element. The engine only learns its node numbers during
    /// [`Circuit::analyze`], so elements may be added in any order.
    pub fn add(&mut self, element: Box<dyn Element>) {
        self.elements.push(element);
    }

    pub fn elements(&self) -> &[Box<dyn Element>] {
        &self.elements
    }

    pub fn elements_mut(&mut self) -> &mut [Box<dyn Element>] {
        &mut self.elements
    }

    pub fn len(&self) -> usize {
        self.elements.len()
    }

    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Number of circuit nodes including ground.
    pub fn node_count(&self) -> usize {
        self.node_list.len()
    }

    /// Nodes the last analysis found to have no conductive path to ground.
    ///
    /// Each is tied to ground through a large resistor so the matrix stays solvable, which is
    /// why an open switch leaves its far node in this list rather than at a floating voltage.
    pub fn unconnected_nodes(&self) -> &[usize] {
        &self.unconnected_nodes
    }

    /// Solved voltage of node `n` (1-based; node 0 is ground and reads 0).
    pub fn node_voltage(&self, n: usize) -> f64 {
        if n == 0 {
            0.0
        } else {
            self.node_voltages.get(n - 1).copied().unwrap_or(0.0)
        }
    }

    /// The node number attached to `p`, if any element has a post on it.
    ///
    /// Node numbers are handed out in the order posts are first seen, so they are not
    /// stable across edits to the element list. Anything that wants to name a node should
    /// name the point it sits on instead and look it up here.
    pub fn node_at_point(&self, p: Point) -> Option<usize> {
        self.elements.iter().find_map(|e| {
            (0..e.post_count())
                .find(|&k| e.post(k) == p)
                .map(|k| e.node(k))
        })
    }

    /// Returns the first element of the given kind, for inspection in tests and UIs.
    pub fn stop(&mut self, err: SimError, element: Option<usize>) {
        self.error = Some(err);
        self.error_element = element;
        self.analysed = false;
    }

    fn clear_error(&mut self) {
        self.error = None;
        self.error_element = None;
    }

    // --- analysis ---------------------------------------------------------

    /// Port of `analyzeCircuit`. Must be re-run whenever the topology changes.
    pub fn analyze(&mut self) -> Result<(), SimError> {
        self.clear_error();
        if self.elements.is_empty() {
            self.node_list.clear();
            self.analysed = false;
            return Ok(());
        }
        self.node_list.clear();
        self.post_count_map.clear();

        self.calculate_wire_closure();
        self.set_ground_node();
        self.make_node_list();

        if !self.calc_wire_info() {
            return Err(self.error.take().unwrap_or(SimError::WireLoop));
        }
        self.wire_groups.clear();
        self.group_node.clear();

        // Assign voltage-source rows and detect whether the circuit is nonlinear.
        self.voltage_sources.clear();
        self.circuit_non_linear = false;
        for i in 0..self.elements.len() {
            if self.elements[i].non_linear() {
                self.circuit_non_linear = true;
            }
            let vsc = self.elements[i].voltage_source_count();
            for j in 0..vsc {
                self.voltage_sources.push(i);
                let vs_index = self.voltage_sources.len() - 1;
                self.elements[i].set_voltage_source(j, vs_index);
            }
        }

        self.find_unconnected_nodes();
        self.validate_circuit()?;
        self.time_step = self.max_time_step;
        self.stamp_circuit()?;
        self.analysed = true;
        Ok(())
    }

    /// Merges the two ends of every wire into a single node identity.
    /// Port of `calculateWireClosure`.
    fn calculate_wire_closure(&mut self) {
        self.wire_groups.clear();
        self.group_node.clear();
        self.wire_info_list.clear();
        for i in 0..self.elements.len() {
            if !self.elements[i].is_wire() {
                continue;
            }
            self.wire_info_list.push(WireInfo {
                wire: i,
                post: 0,
                neighbors: Vec::new(),
            });
            let p0 = self.elements[i].post(0);
            let p1 = self.elements[i].post(1);
            match (
                self.wire_groups.get(&p0).copied(),
                self.wire_groups.get(&p1).copied(),
            ) {
                (Some(a), Some(b)) if a != b => {
                    // Two previously separate groups now touch: fold b into a, repointing
                    // every post that referenced it.
                    for e in self.wire_groups.values_mut() {
                        if *e == b {
                            *e = a;
                        }
                    }
                }
                (Some(a), _) => {
                    self.wire_groups.insert(p1, a);
                }
                (_, Some(b)) => {
                    self.wire_groups.insert(p0, b);
                }
                (None, None) => {
                    let g = self.group_node.len();
                    self.group_node.push(None);
                    self.wire_groups.insert(p0, g);
                    self.wire_groups.insert(p1, g);
                }
            }
        }
    }

    /// Works out which neighbouring elements each wire should sum to get its current.
    ///
    /// Port of `calcWireInfo`. A wire's current can only be computed once every element
    /// touching its reference terminal has a known current. Wires are therefore processed
    /// in dependency order: if a wire is not ready, it is moved to the back of the list
    /// and retried. Detecting a cycle (which would mean the wire currents are mutually
    /// dependent and therefore undefined) is what the move budget guards against.
    fn calc_wire_info(&mut self) -> bool {
        let mut moved = 0usize;
        let mut ready: HashSet<usize> = HashSet::new();
        let mut i = 0usize;
        while i < self.wire_info_list.len() {
            let wire = self.wire_info_list[i].wire;
            // Both ends of a wire share a node number, so any end works. `node_list` is
            // indexed *by* node number: index 0 is the spare ground slot and node n is at
            // index n, so there is no off-by-one to apply here.
            let cn1_index = self.elements[wire].node(0);
            if cn1_index == 0 || cn1_index >= self.node_list.len() {
                self.wire_info_list.swap_remove(i);
                continue;
            }
            let links = self.node_list[cn1_index].links.clone();

            let wire_p0 = self.elements[wire].post(0);
            let wire_p1 = self.elements[wire].post(1);
            let mut neighbors0 = Vec::new();
            let mut neighbors1 = Vec::new();
            let mut is_ready0 = true;
            let mut is_ready1 = true;

            for (elm, num) in links {
                if elm == wire {
                    continue;
                }
                let pt = self.elements[elm].post(num);
                // A neighbouring wire that has not been assigned an end yet would create a
                // circular dependency: it would need our current to compute its own, and we
                // would need its end to know which terminal to add up.
                let not_ready = self.elements[elm].is_wire() && !ready.contains(&elm);
                if pt == wire_p0 {
                    neighbors0.push((elm, num));
                    is_ready0 &= !not_ready;
                } else if pt == wire_p1 {
                    neighbors1.push((elm, num));
                    is_ready1 &= !not_ready;
                }
            }

            if is_ready0 {
                self.wire_info_list[i].neighbors = neighbors0;
                self.wire_info_list[i].post = 0;
                ready.insert(wire);
                moved = 0;
                i += 1;
            } else if is_ready1 {
                self.wire_info_list[i].neighbors = neighbors1;
                self.wire_info_list[i].post = 1;
                ready.insert(wire);
                moved = 0;
                i += 1;
            } else {
                // Retry later. Rotating the list keeps the move budget meaningful.
                let wi = self.wire_info_list.remove(i);
                self.wire_info_list.push(wi);
                moved += 1;
                if moved > self.wire_info_list.len() * 2 {
                    self.stop(SimError::WireLoop, Some(wire));
                    return false;
                }
            }
        }
        true
    }

    /// Chooses node 0: an explicit ground if present, otherwise the first voltage source's
    /// positive terminal, otherwise a fresh isolated node. Port of `setGroundNode`.
    fn set_ground_node(&mut self) {
        let mut got_ground = false;
        let mut got_rail = false;
        let mut volt: Option<usize> = None;

        for i in 0..self.elements.len() {
            let ce = &self.elements[i];
            if ce.is_ground() {
                got_ground = true;
                break;
            }
            if ce.is_rail() {
                got_rail = true;
            }
            if volt.is_none() && ce.is_voltage_source() {
                volt = Some(i);
            }
        }

        let can_use_volt = !got_ground && volt.is_some() && !got_rail;
        let anchor = if can_use_volt {
            Some(self.elements[volt.unwrap()].post(0))
        } else {
            None
        };

        // Node 0 always exists, whether or not a post was claimed for it.
        self.node_list.push(CircuitNode::default());

        if let Some(pt) = anchor {
            match self.wire_groups.get(&pt).copied() {
                Some(g) => self.group_node[g] = Some(0),
                None => {
                    let g = self.group_node.len();
                    self.group_node.push(Some(0));
                    self.wire_groups.insert(pt, g);
                }
            }
        }
    }

    /// Allocates a node number for every post and hidden node.
    /// Port of `makeNodeList`.
    fn make_node_list(&mut self) {
        for i in 0..self.elements.len() {
            let posts = self.elements[i].post_count();
            let inodes = self.elements[i].internal_node_count();

            for j in 0..posts {
                let pt = self.elements[i].post(j);
                *self.post_count_map.entry(pt).or_insert(0) += 1;

                // A post already covered by a wire group takes that group's node; the
                // first post of a group is what allocates it, so a whole run of wires ends
                // up as one node without contributing a single matrix row.
                let n = match self.wire_groups.get(&pt).copied() {
                    Some(g) => match self.group_node[g] {
                        Some(n) => {
                            self.node_list[n].links.push((i, j));
                            n
                        }
                        None => {
                            let n = self.node_list.len();
                            self.group_node[g] = Some(n);
                            self.node_list.push(CircuitNode {
                                links: vec![(i, j)],
                                internal: false,
                            });
                            n
                        }
                    },
                    None => {
                        // A post not covered by any wire group is a group of one. Recording
                        // it here is what makes a *second* post at the same point reuse the
                        // node: Java relies on the shared `nodeMap` entry for exactly this.
                        let g = self.group_node.len();
                        let n = self.node_list.len();
                        self.group_node.push(Some(n));
                        self.wire_groups.insert(pt, g);
                        self.node_list.push(CircuitNode {
                            links: vec![(i, j)],
                            internal: false,
                        });
                        n
                    }
                };
                self.elements[i].set_node(j, n);
                // Ground's voltage is never set by the solver, so pin it now.
                if n == 0 {
                    self.elements[i].set_node_voltage(j, 0.0);
                }
            }

            for j in 0..inodes {
                let n = self.node_list.len();
                self.node_list.push(CircuitNode {
                    links: vec![(i, j + posts)],
                    internal: true,
                });
                self.elements[i].set_node(j + posts, n);
            }
        }
    }

    /// Finds nodes with no conductive path to ground. Port of `findUnconnectedNodes`.
    ///
    /// Such nodes would make the matrix singular, because their KCL row has no reference.
    /// Each is collected so it can be tied to ground through a large resistor.
    fn find_unconnected_nodes(&mut self) {
        let n = self.node_list.len();
        let mut closure = vec![false; n];
        let mut unconnected = Vec::new();
        closure[0] = true;

        loop {
            let mut changed = false;
            for i in 0..self.elements.len() {
                if self.elements[i].is_wire() {
                    continue;
                }
                let count = self.elements[i].connection_node_count();
                for j in 0..count {
                    let nj = self.elements[i].connection_node(j);
                    if nj == 0 || nj >= n {
                        continue;
                    }
                    if !closure[nj] {
                        if self.elements[i].has_ground_connection(j) {
                            closure[nj] = true;
                            changed = true;
                        }
                        continue;
                    }
                    for k in 0..count {
                        if j == k {
                            continue;
                        }
                        let nk = self.elements[i].connection_node(k);
                        if nk == 0 || nk >= n {
                            continue;
                        }
                        if self.elements[i].get_connection(j, k) && !closure[nk] {
                            closure[nk] = true;
                            changed = true;
                        }
                    }
                }
            }
            if changed {
                continue;
            }
            // Tie off one unconnected node at a time and re-run closure, which can
            // reveal further islands.
            let mut found = None;
            for i in 0..n {
                if !closure[i] && !self.node_list[i].internal {
                    found = Some(i);
                    break;
                }
            }
            match found {
                Some(i) => {
                    unconnected.push(i);
                    closure[i] = true;
                }
                None => break,
            }
        }
        self.unconnected_nodes = unconnected;
    }

    /// Rejects circuits that cannot be simulated, and neutralises the ones that only need
    /// a nudge. Port of `validateCircuit`.
    fn validate_circuit(&mut self) -> Result<(), SimError> {
        for i in 0..self.elements.len() {
            // A current source with no path that avoids other current sources has no
            // defined voltage across it.
            if self.elements[i].is_current_source() {
                let dest = self.elements[i].node(1);
                let start = self.elements[i].node(0);
                let broken = !self.find_path(PathType::Induct, i, start, dest);
                self.elements[i].set_broken(broken);
            }

            // A loop of voltage sources and wire-like parts has no resistance, so the
            // source currents are unsolvable. Real wires are exempt: they are folded into
            // the node list before this runs, so they are not the loop, just the wire in it.
            if self.elements[i].post_count() == 2
                && (self.elements[i].is_voltage_source() || self.elements[i].is_wire_like())
            {
                let n0 = self.elements[i].node(0);
                let n1 = self.elements[i].node(1);
                if self.find_path(PathType::Voltage, i, n1, n0) {
                    return Err(SimError::VoltageLoop);
                }
            }

            // A supply rail shorted straight to ground.
            if self.elements[i].is_rail() {
                let n0 = self.elements[i].node(0);
                if self.find_path(PathType::Voltage, i, n0, 0) {
                    return Err(SimError::RailToGround);
                }
            }

            if self.elements[i].is_capacitor() {
                let n0 = self.elements[i].node(0);
                let n1 = self.elements[i].node(1);
                if self.find_path(PathType::Short, i, n1, n0) {
                    // A shorted capacitor must drop its stored voltage or it rings forever.
                    self.elements[i].shorted();
                } else if self.find_path(PathType::CapV, i, n1, n0) {
                    return Err(SimError::CapacitorLoop);
                }
            }
        }
        Ok(())
    }

    /// Depth-first search for a path from `n1` back to `dest` obeying `kind`'s
    /// connectivity rules. Port of `FindPathInfo.findPath`.
    fn find_path(&self, kind: PathType, first_elm: usize, n1: usize, dest: usize) -> bool {
        let n = self.node_list.len();
        if n1 == dest {
            return true;
        }
        if n1 >= n {
            return false;
        }
        let mut visited = vec![false; n];
        self.find_path_rec(kind, first_elm, n1, dest, &mut visited)
    }

    fn find_path_rec(
        &self,
        kind: PathType,
        first_elm: usize,
        n1: usize,
        dest: usize,
        visited: &mut [bool],
    ) -> bool {
        if n1 == dest {
            return true;
        }
        if n1 >= self.node_list.len() || visited[n1] {
            return false;
        }
        visited[n1] = true;

        for i in 0..self.elements.len() {
            if i == first_elm {
                continue;
            }
            let ce = &self.elements[i];

            // Inductors and current sources need a path free of current sources.
            if kind == PathType::Induct && ce.is_current_source() {
                continue;
            }
            // When checking for voltage loops, only wires, sources and ground count: a loop
            // through anything with resistance is fine. `is_wire_like` rather than `is_wire`
            // because a closed switch conducts like a wire here without being one.
            if kind == PathType::Voltage
                && !(ce.is_wire_like() || ce.is_voltage_source() || ce.is_ground())
            {
                continue;
            }
            if kind == PathType::Short && !ce.is_wire_like() {
                continue;
            }
            if kind == PathType::CapV
                && !(ce.is_wire_like() || ce.is_capacitor() || ce.is_voltage_source())
            {
                continue;
            }

            let count = ce.connection_node_count();

            // While standing on ground, any element with a grounded post gives us a way to
            // step off it, because a path may pass through ground.
            if n1 == 0 {
                for j in 0..count {
                    if ce.has_ground_connection(j)
                        && self.find_path_rec(kind, first_elm, ce.connection_node(j), dest, visited)
                    {
                        return true;
                    }
                }
            }

            // The element only matters if it actually touches the node we are standing on.
            let Some(j) = (0..count).find(|&j| ce.connection_node(j) == n1) else {
                continue;
            };

            // Stepping through ground is always an option, even from a non-ground node.
            if ce.has_ground_connection(j) && self.find_path_rec(kind, first_elm, 0, dest, visited)
            {
                return true;
            }

            // Parallel inductors only count as a conducting path if they already carry the
            // same current, so a bank of uncoupled inductors is not treated as a short.
            if kind == PathType::Induct && ce.is_inductor() {
                let mut c = ce.current();
                if j == 0 {
                    c = -c;
                }
                if (c - self.elements[first_elm].current()).abs() > 1e-10 {
                    continue;
                }
            }

            for k in 0..count {
                if j == k {
                    continue;
                }
                if ce.get_connection(j, k)
                    && self.find_path_rec(kind, first_elm, ce.connection_node(k), dest, visited)
                {
                    return true;
                }
            }
        }
        false
    }

    // --- stamping ---------------------------------------------------------

    /// Allocates the system matrix and has every element stamp itself.
    /// Port of `stampCircuit`.
    pub fn stamp_circuit(&mut self) -> Result<(), SimError> {
        if self.elements.is_empty() {
            return Ok(());
        }
        let vs_count = self.voltage_sources.len();
        self.stamper.reset(self.node_list.len(), vs_count);
        self.node_voltages = vec![0.0; self.node_list.len() - 1];
        self.last_node_voltages = vec![0.0; self.node_list.len() - 1];

        // Refresh the context elements read while stamping.
        self.ctx.time_step = self.time_step;
        self.ctx.time = self.time;
        self.ctx.dc_analysis = self.dc_analysis;
        // Voltage sources address their own row as `node_count + vs`, so elements need the
        // ground-inclusive node count to compute it.
        self.ctx.node_count = self.node_list.len();

        // Tie off floating nodes before anything else stamps.
        let unconnected = std::mem::take(&mut self.unconnected_nodes);
        for n in unconnected {
            self.stamper.stamp_resistor(0, n, 1e8);
        }

        let Circuit {
            elements,
            stamper,
            ctx,
            ..
        } = self;
        for e in elements.iter_mut() {
            e.stamp(stamper, ctx);
        }

        self.stamper.simplify().map_err(|e| match e {
            StamperError::MatrixError => SimError::MatrixError,
            StamperError::SingularMatrix => SimError::SingularMatrix,
        })?;

        // A linear circuit's factorisation never changes, so do it once here and skip the
        // per-timestep refactorisation entirely.
        if !self.circuit_non_linear {
            self.stamper.solve().map_err(|_| SimError::SingularMatrix)?;
        }
        Ok(())
    }

    // --- stepping ---------------------------------------------------------

    /// Advances the simulation by one timestep. Port of `runCircuit`.
    pub fn step(&mut self) -> Result<(), SimError> {
        if !self.analysed || self.error.is_some() {
            return Err(self.error.clone().unwrap_or(SimError::MatrixError));
        }
        if self.elements.is_empty() {
            self.time += self.time_step;
            return Ok(());
        }

        // A few iterations without convergence trouble in a row earn the right to double
        // the timestep. Both counters live on the struct because they persist across
        // timesteps, exactly as `CirSim.goodIterations`/`goodIteration` do.
        self.good_iteration = false;
        loop {
            if self.good_iterations >= 3
                && self.time_step < self.max_time_step
                && self.good_iteration
            {
                self.time_step = (self.time_step * 2.0).min(self.max_time_step);
                self.stamp_circuit()?;
                self.good_iterations = 0;
            }

            // The context has to be refreshed every timestep, not only when the circuit is
            // re-stamped. A time-varying source reads `ctx.time` in `do_step`, and
            // `stamp_circuit` is only reached when the topology or the timestep changes — so
            // without this a sine source would keep emitting the value it had at the last
            // re-stamp, which for a source that starts at zero phase is silence.
            self.ctx.time_step = self.time_step;
            self.ctx.time = self.time;

            for e in self.elements.iter_mut() {
                e.start_iteration(&self.ctx);
            }
            self.steps += 1;

            let subiter_count =
                if self.adjust_time_step && self.time_step / 2.0 > self.min_time_step {
                    100
                } else {
                    5000
                };

            // Newton/companion-model iteration. `subiter` is incremented at the *end* of
            // the body so that breaking early leaves it at the iteration that converged,
            // while running out of iterations leaves it at `subiter_count` — which is what
            // the convergence check below distinguishes.
            let mut subiter = 0usize;
            while subiter != subiter_count {
                self.ctx.set_converged(true);
                self.ctx.sub_iterations.set(subiter);
                self.stamper.restore(self.circuit_non_linear);

                let Circuit {
                    elements,
                    stamper,
                    ctx,
                    ..
                } = self;
                for e in elements.iter_mut() {
                    e.do_step(stamper, ctx);
                }

                // A companion model that has blown up poisons the whole matrix; catching it
                // here gives a clean error instead of a NaN propagating into the solution.
                if !self.stamper.matrix_is_finite() {
                    return Err(SimError::NonFinite);
                }

                if self.circuit_non_linear {
                    // Elements signal convergence from `do_step`. The first pass is always
                    // taken so a solution has a chance to settle before we judge it.
                    if self.ctx.converged() && subiter > 0 {
                        break;
                    }
                    // The matrix changed, so the previous factors are stale.
                    self.stamper
                        .factor()
                        .map_err(|_| SimError::SingularMatrix)?;
                }
                self.stamper.solve_factored();
                self.apply_solved_right_side();
                if !self.circuit_non_linear {
                    break;
                }
                subiter += 1;
            }
            self.sub_iterations = subiter;

            if subiter == subiter_count {
                // Convergence failed. Halve the step and re-stamp, unless we are already at
                // the floor.
                self.good_iterations = 0;
                if self.adjust_time_step {
                    self.time_step /= 2.0;
                }
                if self.time_step < self.min_time_step || !self.adjust_time_step {
                    return Err(SimError::ConvergenceFailed);
                }
                // Undo the failed attempt before retrying with the smaller step.
                self.node_voltages.copy_from_slice(&self.last_node_voltages);
                self.stamp_circuit()?;
                continue;
            }

            if subiter < 3 {
                self.good_iterations += 1;
            } else {
                self.good_iterations = 0;
            }
            self.time += self.time_step;
            self.good_iteration = true;

            for e in self.elements.iter_mut() {
                e.step_finished(&self.ctx);
            }
            if self.ctx.stop_requested() {
                self.error = Some(SimError::MaxCurrentExceeded);
                return Err(SimError::MaxCurrentExceeded);
            }
            self.calc_wire_currents();
            self.last_node_voltages.copy_from_slice(&self.node_voltages);
            return Ok(());
        }
    }

    /// Publishes a solved right-hand side back onto the elements.
    /// Port of `applySolvedRightSide`.
    ///
    /// The solved vector only covers the *simplified* matrix, so each full-system row is
    /// mapped back through its [`RowInfo`]: a row that simplification turned into a
    /// constant reports that constant, and a live row reports its column of the solution.
    fn apply_solved_right_side(&mut self) {
        let rs = self.stamper.rhs().to_vec();
        let full = self.stamper.full_size();
        let node_count = self.node_list.len();
        for j in 0..full {
            let ri = &self.stamper.row_info()[j];
            let res = if ri.row_type == RowType::Const {
                ri.value
            } else {
                match ri.map_col {
                    Some(c) => rs.get(c).copied().unwrap_or(0.0),
                    None => 0.0,
                }
            };
            if res.is_nan() {
                self.ctx.set_converged(false);
                break;
            }
            if j + 1 < node_count {
                self.node_voltages[j] = res;
            } else {
                let ji = j + 1 - node_count;
                if let Some(&elm) = self.voltage_sources.get(ji) {
                    self.elements[elm].set_current(ji, res);
                }
            }
        }
        self.set_node_voltages();
    }

    /// Pushes node voltages onto every element attached to each node.
    /// Port of `setNodeVoltages`.
    fn set_node_voltages(&mut self) {
        for j in 0..self.node_voltages.len() {
            let res = self.node_voltages[j];
            let node = j + 1;
            let Some(links) = self.node_list.get(node).map(|cn| cn.links.clone()) else {
                continue;
            };
            for (elm, num) in links {
                self.elements[elm].set_node_voltage(num, res);
            }
        }
    }

    /// Recomputes wire currents from the currents of the elements touching each wire.
    /// Port of `calcWireCurrents`.
    ///
    /// Each wire is assigned one end during analysis (see [`Circuit::calc_wire_info`]). The
    /// current there is the sum of what the neighbouring elements at that point carry
    /// *into* it, by KCL.
    ///
    /// The neighbour tuples already record which post of the neighbour shares the point, so
    /// that is what gets passed to `current_into_node` — it wants a post index, not a node
    /// number, and the two only coincide for a two-terminal element's first post.
    fn calc_wire_currents(&mut self) {
        for wi in &self.wire_info_list {
            let wire = wi.wire;
            let mut cur = 0.0;
            for &(elm, num) in &wi.neighbors {
                cur += self.elements[elm].current_into_node(num);
            }
            // The reference end carries the summed current; the far end is its negation.
            let sign = if wi.post == 0 { 1.0 } else { -1.0 };
            self.elements[wire].set_current(0, sign * cur);
        }
    }

    /// Re-runs analysis, picking up any topology or parameter change.
    pub fn reanalyze(&mut self) -> Result<(), SimError> {
        self.analyze()
    }

    /// Resets all state to power-on values. Port of `resetAction`.
    pub fn reset(&mut self) {
        self.time = 0.0;
        self.steps = 0;
        for e in self.elements.iter_mut() {
            e.reset();
        }
        for v in self.node_voltages.iter_mut() {
            *v = 0.0;
        }
        for v in self.last_node_voltages.iter_mut() {
            *v = 0.0;
        }
    }
}
