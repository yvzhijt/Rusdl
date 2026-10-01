//! MNA stamping: the matrix, right-hand side, and the row/column bookkeeping that lets
//! circuitjs1 eliminate trivially-solvable rows before factorising.
//!
//! Port of the stamping helpers (`stampMatrix`, `stampResistor`, `stampVCVS`,
//! `stampCCCS`, `stampVCCurrentSource`, `stampCurrentSource`, `stampVoltageSource`,
//! `updateVoltageSource`, `stampNonLinear`, `stampRightSide`) and `simplifyMatrix`
//! from `CirSim.java` (lines 2173-2280 and 2423-2546).

use crate::lu::{lu_factor, lu_solve, Matrix};

/// How a matrix row behaves once simplification has run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RowType {
    /// Ordinary row: the unknown stays in the matrix.
    Normal,
    /// The row collapsed to a single non-constant unknown, so the unknown's value is a
    /// compile-time constant that must be folded back into every other row.
    Const,
}

/// Per-row bookkeeping for matrix simplification.
///
/// Port of `RowInfo.java`.
#[derive(Clone, Debug)]
pub struct RowInfo {
    pub row_type: RowType,
    /// Column index in the simplified matrix, if this row is `Normal` and kept.
    pub map_col: Option<usize>,
    /// Row index in the simplified matrix, if this row is kept.
    pub map_row: Option<usize>,
    /// The constant value, when `row_type == Const`.
    pub value: f64,
    /// The row's right-hand side is rewritten every `doStep` (dynamic source).
    pub rs_changes: bool,
    /// The row's left-hand side is rewritten every `doStep` (nonlinear element).
    pub ls_changes: bool,
    /// The row has been folded away entirely.
    pub drop_row: bool,
}

impl Default for RowInfo {
    fn default() -> Self {
        Self {
            row_type: RowType::Normal,
            map_col: None,
            map_row: None,
            value: 0.0,
            rs_changes: false,
            ls_changes: false,
            drop_row: false,
        }
    }
}

impl RowInfo {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Errors raised while building the system matrix.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StamperError {
    /// `qp == -1`: a live row had no non-constant entry left after elimination.
    MatrixError,
    /// `lu_factor` rejected the matrix (an all-zero row).
    SingularMatrix,
}

/// A single row-elimination step found by [`Stamper::find_removal`].
#[derive(Clone, Copy, Debug)]
enum Removal {
    /// Row `row` has a single non-constant entry in column `col`, so `col` becomes a
    /// constant and the row is dropped.
    MakeConst { col: usize, row: usize, value: f64 },
    /// Row `row` was already reduced to a constant and is now redundant.
    DropRow { row: usize },
}

/// Owns the MNA system matrix and performs all stamping.
///
/// Index convention, inherited unchanged from the Java original:
///
/// * **nodes** are 1-based; node `n` occupies row/column `n - 1`. Node 0 is ground and
///   is never stamped, which is what makes `[i > 0 && j > 0]` guard work.
/// * **voltage sources** occupy row/column `node_count + vs - 1`, i.e. they are addressed
///   with the 1-based value `node_count + vs` and the `-1` happens inside `stamp_matrix`.
///
/// Keeping the 1-based convention rather than "fixing" it means element code translates
/// directly from the Java `stamp*` calls.
pub struct Stamper {
    matrix: Matrix,
    rhs: Vec<f64>,
    orig_matrix: Matrix,
    orig_rhs: Vec<f64>,
    row_info: Vec<RowInfo>,
    permute: Vec<u32>,
    /// Size of the (possibly simplified) matrix actually being solved.
    size: usize,
    /// Size before simplification: the row count of the full system.
    full_size: usize,
    /// Number of circuit nodes including ground, i.e. `node_list.len()`.
    node_count: usize,
    needs_map: bool,
}

impl Stamper {
    /// Allocates the system for `node_count` nodes (ground included) and
    /// `voltage_source_count` extra rows.
    pub fn new(node_count: usize, voltage_source_count: usize) -> Self {
        let n = node_count - 1 + voltage_source_count;
        Self {
            matrix: Matrix::zeros(n),
            rhs: vec![0.0; n],
            orig_matrix: Matrix::zeros(n),
            orig_rhs: vec![0.0; n],
            row_info: (0..n).map(|_| RowInfo::default()).collect(),
            permute: vec![0u32; n],
            size: n,
            full_size: n,
            node_count,
            needs_map: false,
        }
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn full_size(&self) -> usize {
        self.full_size
    }

    pub fn node_count(&self) -> usize {
        self.node_count
    }

    pub fn matrix(&self) -> &Matrix {
        &self.matrix
    }

    pub fn rhs(&self) -> &[f64] {
        &self.rhs
    }

    pub fn rhs_mut(&mut self) -> &mut [f64] {
        &mut self.rhs
    }

    pub fn row_info(&self) -> &[RowInfo] {
        &self.row_info
    }

    // --- stamping ---------------------------------------------------------

    /// Adds `x` to the coefficient of `x_j` in the equation for `i`.
    ///
    /// Semantics: a voltage change of `dv` at column `j` increases the current into
    /// node `i` by `x * dv`. Once simplification has run, a constant column is folded
    /// straight into the right-hand side instead of the matrix.
    ///
    /// Port of `stampMatrix`.
    #[inline]
    pub fn stamp_matrix(&mut self, i: usize, j: usize, x: f64) {
        if i == 0 || j == 0 {
            return;
        }
        let (i, j) = if self.needs_map {
            let Some(row) = self.row_info[i - 1].map_row else {
                // The row was folded away. circuitjs1 would index -1 here and throw;
                // rows in this state are unreachable from element stamps, so skipping
                // is the safe equivalent of the original's crash.
                return;
            };
            if self.row_info[j - 1].row_type == RowType::Const {
                let v = self.row_info[j - 1].value;
                self.rhs[row] -= x * v;
                return;
            }
            match self.row_info[j - 1].map_col {
                Some(col) => (row, col),
                None => return,
            }
        } else {
            (i - 1, j - 1)
        };
        self.matrix.add(i, j, x);
    }

    /// Adds `x` to the right-hand side of row `i` (an independent current flowing into
    /// node `i`). Port of `stampRightSide(int, double)`.
    #[inline]
    pub fn stamp_right_side(&mut self, i: usize, x: f64) {
        if i == 0 {
            return;
        }
        let row = if self.needs_map {
            match self.row_info[i - 1].map_row {
                Some(r) => r,
                None => return,
            }
        } else {
            i - 1
        };
        self.rhs[row] += x;
    }

    /// Marks row `i`'s right-hand side as rewritten on every `doStep`.
    /// Port of `stampRightSide(int)`.
    #[inline]
    pub fn mark_rhs_changes(&mut self, i: usize) {
        if i > 0 {
            self.row_info[i - 1].rs_changes = true;
        }
    }

    /// Marks row `i`'s left-hand side as rewritten on every `doStep`.
    /// Port of `stampNonLinear`.
    #[inline]
    pub fn mark_non_linear(&mut self, i: usize) {
        if i > 0 {
            self.row_info[i - 1].ls_changes = true;
        }
    }

    /// `1/r` between `n1` and `n2`. Port of `stampResistor`.
    pub fn stamp_resistor(&mut self, n1: usize, n2: usize, r: f64) {
        self.stamp_conductance(n1, n2, 1.0 / r);
    }

    /// Conductance `g` between `n1` and `n2`. Port of `stampConductance`.
    pub fn stamp_conductance(&mut self, n1: usize, n2: usize, g: f64) {
        self.stamp_matrix(n1, n1, g);
        self.stamp_matrix(n2, n2, g);
        self.stamp_matrix(n1, n2, -g);
        self.stamp_matrix(n2, n1, -g);
    }

    /// Independent current source `i` from `n1` to `n2`.
    /// Port of `stampCurrentSource`.
    pub fn stamp_current_source(&mut self, n1: usize, n2: usize, i: f64) {
        self.stamp_right_side(n1, -i);
        self.stamp_right_side(n2, i);
    }

    /// Voltage-controlled voltage source: control voltage from `n1` to `n2`, scaled by
    /// `coef`, sharing equation row `vs` (which must also be stamped by
    /// [`Stamper::stamp_voltage_source`]). Port of `stampVCVS`.
    pub fn stamp_vcvs(&mut self, n1: usize, n2: usize, coef: f64, vs: usize) {
        let vn = self.node_count + vs;
        self.stamp_matrix(vn, n1, coef);
        self.stamp_matrix(vn, n2, -coef);
    }

    /// Voltage source `vs` from `n1` to `n2` with value `v`.
    /// Port of `stampVoltageSource(n1, n2, vs, v)`.
    pub fn stamp_voltage_source(&mut self, n1: usize, n2: usize, vs: usize, v: f64) {
        let vn = self.node_count + vs;
        self.stamp_matrix(vn, n1, -1.0);
        self.stamp_matrix(vn, n2, 1.0);
        self.stamp_right_side(vn, v);
        self.stamp_matrix(n1, vn, 1.0);
        self.stamp_matrix(n2, vn, -1.0);
    }

    /// Voltage source `vs` whose value is supplied later by
    /// [`Stamper::update_voltage_source`] from `doStep`. Port of
    /// `stampVoltageSource(n1, n2, vs)`.
    pub fn stamp_voltage_source_dynamic(&mut self, n1: usize, n2: usize, vs: usize) {
        let vn = self.node_count + vs;
        self.stamp_matrix(vn, n1, -1.0);
        self.stamp_matrix(vn, n2, 1.0);
        self.mark_rhs_changes(vn);
        self.stamp_matrix(n1, vn, 1.0);
        self.stamp_matrix(n2, vn, -1.0);
    }

    /// Update a dynamic voltage source's value during `doStep`. Port of
    /// `updateVoltageSource`.
    ///
    /// `n1`/`n2` are part of the Java signature for symmetry but unused: the left-hand side
    /// was already stamped by [`Stamper::stamp_voltage_source_dynamic`] and only the
    /// right-hand side needs rewriting.
    pub fn update_voltage_source(&mut self, _n1: usize, _n2: usize, vs: usize, v: f64) {
        let vn = self.node_count + vs;
        self.stamp_right_side(vn, v);
    }

    /// Current source from `cn1` to `cn2` equal to `(V(vn1) - V(vn2)) / g`.
    /// Port of `stampVCCurrentSource`.
    pub fn stamp_vc_current_source(
        &mut self,
        cn1: usize,
        cn2: usize,
        vn1: usize,
        vn2: usize,
        g: f64,
    ) {
        self.stamp_matrix(cn1, vn1, g);
        self.stamp_matrix(cn2, vn2, g);
        self.stamp_matrix(cn1, vn2, -g);
        self.stamp_matrix(cn2, vn1, -g);
    }

    /// Current source from `n1` to `n2` proportional to the current through `vs`.
    /// Port of `stampCCCS`.
    pub fn stamp_cccs(&mut self, n1: usize, n2: usize, vs: usize, gain: f64) {
        let vn = self.node_count + vs;
        self.stamp_matrix(n1, vn, gain);
        self.stamp_matrix(n2, vn, -gain);
    }

    // --- solve ------------------------------------------------------------

    /// Restores `matrix`/`rhs` from the values captured by the last successful
    /// [`Stamper::simplify`]. Called at the top of every Newton sub-iteration.
    ///
    /// `restore_matrix` mirrors the Java original's `if (circuitNonLinear)` guard: linear
    /// elements only ever rewrite the right-hand side, so for a linear circuit the matrix
    /// is known to be untouched and the copy is skipped.
    pub fn restore(&mut self, restore_matrix: bool) {
        for i in 0..self.size {
            self.rhs[i] = self.orig_rhs[i];
        }
        if restore_matrix {
            self.matrix.clone_from(&self.orig_matrix);
        }
    }

    /// Factors the matrix in place. Must be called before [`Stamper::solve_factored`]
    /// whenever the matrix can change between solves.
    pub fn factor(&mut self) -> Result<(), StamperError> {
        if !lu_factor(&mut self.matrix, &mut self.permute) {
            return Err(StamperError::SingularMatrix);
        }
        Ok(())
    }

    /// Solves using the factors currently in the matrix, writing the solution into `rhs`.
    pub fn solve_factored(&mut self) {
        lu_solve(&self.matrix, &self.permute, &mut self.rhs);
    }

    /// Factors then solves. Convenient for callers that always rebuild the matrix.
    pub fn solve(&mut self) -> Result<(), StamperError> {
        self.factor()?;
        self.solve_factored();
        Ok(())
    }

    /// True if the matrix holds a NaN or infinity. `runCircuit` checks this every
    /// sub-iteration before solving, because a NaN poisons the whole state.
    pub fn matrix_is_finite(&self) -> bool {
        !self.matrix.has_non_finite()
    }

    // --- simplification ---------------------------------------------------

    /// Eliminates rows that reduce to a single non-constant unknown.
    ///
    /// This is the single biggest performance lever in circuitjs1: a row whose only
    /// non-zero entry sits in an already-constant column only *defines* that constant,
    /// so the row is dropped and the value is subtracted from every other row that
    /// referenced it. For digital circuits this typically removes the large majority of
    /// rows, because every node tied to a constant voltage rail collapses immediately.
    ///
    /// Each removal turns one column into a constant and drops one row, so the system
    /// stays square. Removal can cascade, so after every successful removal we rescan
    /// from the top — this is the `i = -1` restart in the Java original, expressed as
    /// an outer loop instead of recursion.
    ///
    /// Port of `simplifyMatrix`.
    pub fn simplify(&mut self) -> Result<(), StamperError> {
        let matrix_size = self.size;
        // Every iteration drops at least one row, so this terminates in <= matrix_size
        // passes.
        loop {
            let Some(removal) = self.find_removal(matrix_size)? else {
                break;
            };
            match removal {
                Removal::MakeConst { col, row, value } => {
                    self.row_info[col].row_type = RowType::Const;
                    self.row_info[col].value = value;
                    self.row_info[row].drop_row = true;
                }
                Removal::DropRow { row } => {
                    self.row_info[row].drop_row = true;
                }
            }
        }
        self.finish_simplify();
        Ok(())
    }

    /// Scans every eligible row once and returns the first removal that applies.
    ///
    /// Returns `Ok(None)` when no row can be reduced any further, and
    /// `Err(MatrixError)` when a row that still carries a live unknown has nothing left
    /// to determine it with.
    fn find_removal(&self, matrix_size: usize) -> Result<Option<Removal>, StamperError> {
        for i in 0..matrix_size {
            let re = &self.row_info[i];
            if re.ls_changes || re.drop_row || re.rs_changes {
                continue;
            }

            // Look for rows that can be removed: exactly one non-constant non-zero.
            let mut qp: Option<usize> = None;
            let mut qv = 0.0f64;
            let mut rsadd = 0.0f64;
            let mut multiple = false;
            for j in 0..matrix_size {
                let q = self.matrix.get(i, j);
                if self.row_info[j].row_type == RowType::Const {
                    // Keep a running total of const values already removed.
                    rsadd -= self.row_info[j].value * q;
                    continue;
                }
                if q == 0.0 {
                    continue;
                }
                if qp.is_none() {
                    qp = Some(j);
                    qv = q;
                    continue;
                }
                // More than one nonzero element? Give up on this row.
                multiple = true;
                break;
            }
            if multiple {
                continue;
            }

            return Ok(Some(match qp {
                // The row determines a single unknown, so that unknown is a constant.
                Some(col) => {
                    if self.row_info[col].row_type != RowType::Normal {
                        continue;
                    }
                    Removal::MakeConst {
                        col,
                        row: i,
                        value: (self.rhs[i] + rsadd) / qv,
                    }
                }
                // No non-constant entry left. If this row was itself already reduced to
                // a constant its equation is now redundant, so it can go too. Otherwise a
                // live unknown has no equation at all and the system is degenerate.
                None => {
                    if re.row_type == RowType::Const {
                        Removal::DropRow { row: i }
                    } else {
                        return Err(StamperError::MatrixError);
                    }
                }
            }));
        }
        Ok(None)
    }

    /// Assigns column indices and rebuilds a physically smaller matrix.
    fn finish_simplify(&mut self) {
        let matrix_size = self.size;

        // Find the size of the new matrix.
        let mut nn = 0usize;
        for ri in self.row_info.iter_mut().take(matrix_size) {
            match ri.row_type {
                RowType::Normal => {
                    ri.map_col = Some(nn);
                    nn += 1;
                }
                RowType::Const => {
                    ri.map_col = None;
                }
            }
        }

        // Make the new, simplified matrix.
        let newsize = nn;
        let mut newmatx = Matrix::zeros(newsize);
        let mut newrs = vec![0.0; newsize];
        let mut ii = 0usize;
        for i in 0..matrix_size {
            if self.row_info[i].drop_row {
                self.row_info[i].map_row = None;
                continue;
            }
            newrs[ii] = self.rhs[i];
            self.row_info[i].map_row = Some(ii);
            for j in 0..matrix_size {
                if self.row_info[j].row_type == RowType::Const {
                    newrs[ii] -= self.row_info[j].value * self.matrix.get(i, j);
                } else if let Some(col) = self.row_info[j].map_col {
                    newmatx.add(ii, col, self.matrix.get(i, j));
                }
            }
            ii += 1;
        }

        self.matrix = newmatx;
        self.rhs = newrs;
        self.size = newsize;
        self.orig_matrix.clone_from(&self.matrix);
        self.orig_rhs.clone_from(&self.rhs);
        self.needs_map = true;
    }

    /// Re-allocates and re-captures the baseline system. Used after a timestep change,
    /// which invalidates every companion model.
    pub fn reset(&mut self, node_count: usize, voltage_source_count: usize) {
        let n = node_count - 1 + voltage_source_count;
        self.node_count = node_count;
        self.size = n;
        self.full_size = n;
        self.matrix = Matrix::zeros(n);
        self.rhs = vec![0.0; n];
        self.orig_matrix = Matrix::zeros(n);
        self.orig_rhs = vec![0.0; n];
        self.permute = vec![0u32; n];
        for ri in self.row_info.iter_mut() {
            ri.reset();
        }
        self.row_info.truncate(n);
        while self.row_info.len() < n {
            self.row_info.push(RowInfo::default());
        }
        self.needs_map = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A single node tied to a 10k resistor and a 0V source to ground. Every unknown
    /// collapses to a constant, so simplification must reduce the system to 0x0 and
    /// still report success.
    #[test]
    fn simplifies_away_fully_degenerate_system() {
        let mut s = Stamper::new(2, 1); // 1 non-ground node + 1 voltage source
        s.stamp_resistor(1, 0, 10_000.0);
        s.stamp_voltage_source(0, 1, 0, 0.0);
        s.simplify().expect("simplification should succeed");
        assert_eq!(s.size(), 0);
        assert_eq!(s.full_size(), 2);
        // Node 1 and the source current both collapse to 0 V / 0 A.
        assert_eq!(s.row_info()[0].value, 0.0);
        assert_eq!(s.row_info()[1].value, 0.0);
    }

    /// Node driven by a fixed 5V source through a resistor: the node voltage is 5V and
    /// the source current is -5/R. Simplification should still leave a solvable result.
    #[test]
    fn simplifies_but_preserves_solution() {
        let mut s = Stamper::new(2, 1);
        s.stamp_resistor(1, 0, 1_000.0);
        s.stamp_voltage_source(0, 1, 0, 5.0);
        s.simplify().unwrap();
        s.solve().unwrap();
        // Map the solution back through the row info the way apply_solved_right_side does.
        let mut out = vec![0.0; s.full_size()];
        for (j, ri) in s.row_info().iter().enumerate() {
            out[j] = if ri.row_type == RowType::Const {
                ri.value
            } else {
                s.rhs()[ri.map_col.expect("normal row keeps a column")]
            };
        }
        assert!((out[0] - 5.0).abs() < 1e-9, "node = {}", out[0]);
        // The raw solved source current is +5mA. `VoltageElm.setCurrent` negates it, since
        // the MNA unknown is the current *into* the source's positive terminal.
        assert!((out[1] - 5e-3).abs() < 1e-9, "source current = {}", out[1]);
    }

    /// A row flagged as having a dynamic right-hand side must not be eliminated.
    #[test]
    fn dynamic_rows_are_never_eliminated() {
        let mut s = Stamper::new(2, 1);
        s.stamp_resistor(1, 0, 1_000.0);
        s.stamp_voltage_source_dynamic(0, 1, 0);
        s.update_voltage_source(0, 1, 0, 3.0);
        s.simplify().unwrap();
        s.solve().unwrap();
        let mut out = vec![0.0; s.full_size()];
        for (j, ri) in s.row_info().iter().enumerate() {
            out[j] = if ri.row_type == RowType::Const {
                ri.value
            } else {
                s.rhs()[ri.map_col.expect("normal row keeps a column")]
            };
        }
        assert!((out[0] - 3.0).abs() < 1e-9, "node = {}", out[0]);
    }

    /// Ground (node 0) must never be stamped into: the row and column belonging to the
    /// reference stay entirely zero, so a resistor from ground to node 2 shows up only in
    /// node 2's own diagonal.
    #[test]
    fn ground_index_is_ignored() {
        let mut s = Stamper::new(3, 0); // nodes 1 and 2, so a 2x2 matrix
        s.stamp_resistor(0, 2, 100.0);
        let g = 1.0 / 100.0;
        assert_eq!(s.matrix().row(0), &[0.0, 0.0], "ground row must stay empty");
        assert_eq!(s.matrix().row(1), &[0.0, g]);
    }

    #[test]
    fn all_zero_matrix_is_singular() {
        let mut s = Stamper::new(3, 0);
        assert_eq!(s.solve(), Err(StamperError::SingularMatrix));
    }
}
