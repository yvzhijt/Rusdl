//! Dense row-major matrix plus the Crout LU factorisation used by the MNA solver.
//!
//! Direct port of `lu_factor` / `lu_solve` from
//! `circuit-js/src/com/lushprojects/circuitjs1/client/CirSim.java` (lines 4972-5082).
//!
//! circuitjs1 deliberately does *not* use a general sparse solver: the matrix is small
//! (a few hundred rows at most for digital circuits) and the whole thing is stamped and
//! factored inside a Newton loop, so a dense row-major layout with in-place elimination
//! wins on constant factor. We keep the exact same numerics, including the singular-row
//! pre-scan, the `>=` tie-break in the pivot search, and the `1e-18` zero-pivot fudge, so
//! that Rust results match the Java reference bit for bit on the same circuit.

/// Row-major dense matrix of `f64`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Matrix {
    n: usize,
    data: Vec<f64>,
}

impl Matrix {
    /// Allocates an `n x n` zero matrix.
    pub fn zeros(n: usize) -> Self {
        Self {
            n,
            data: vec![0.0; n * n],
        }
    }

    /// Number of rows (== number of columns).
    pub fn size(&self) -> usize {
        self.n
    }

    /// Row-major storage, `row * n + col`. Used by [`lu_factor`] for tight inner loops.
    #[inline]
    pub fn as_slice(&self) -> &[f64] {
        &self.data
    }

    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.n + col]
    }

    #[inline]
    pub fn set(&mut self, row: usize, col: usize, v: f64) {
        self.data[row * self.n + col] = v;
    }

    /// `A[row][col] += v`. This is the accumulation used by every `stampMatrix` call.
    #[inline]
    pub fn add(&mut self, row: usize, col: usize, v: f64) {
        self.data[row * self.n + col] += v;
    }

    #[inline]
    pub fn row(&self, row: usize) -> &[f64] {
        &self.data[row * self.n..(row + 1) * self.n]
    }

    /// Zeroes every entry in place, keeping the allocation.
    pub fn clear(&mut self) {
        for v in self.data.iter_mut() {
            *v = 0.0;
        }
    }

    #[inline]
    fn swap_rows(&mut self, a: usize, b: usize) {
        if a == b {
            return;
        }
        for col in 0..self.n {
            self.data.swap(a * self.n + col, b * self.n + col);
        }
    }

    /// True if any entry is not finite. `runCircuit` checks this every sub-iteration.
    pub fn has_non_finite(&self) -> bool {
        self.data.iter().any(|v| !v.is_finite())
    }
}

/// In-place Crout LU factorisation with partial (max-magnitude) pivoting.
///
/// Writes the combined L\U factors back into `a` and records the row interchanges in
/// `ipvt`. Returns `false` if the matrix is rejected as singular.
///
/// Port of `lu_factor`. Note the deliberately quirky details that are preserved for
/// numerical parity with the Java original:
/// - the pre-scan rejects any row that is entirely zero, before any elimination happens;
/// - the pivot search uses `>=`, so ties select the *last* maximal row;
/// - a zero pivot is nudged to `1e-18` rather than erroring out.
pub fn lu_factor(a: &mut Matrix, ipvt: &mut [u32]) -> bool {
    let n = a.size();
    debug_assert!(ipvt.len() >= n, "ipvt must be at least n entries");

    if n == 0 {
        return true;
    }

    // Check for a possible singular matrix by scanning for rows that are all zeroes.
    for i in 0..n {
        let row = a.row(i);
        if row.iter().all(|&v| v == 0.0) {
            return false;
        }
    }

    // Crout's method: loop through the columns.
    for j in 0..n {
        // Calculate upper triangular elements for this column.
        for i in 0..j {
            let mut q = a.get(i, j);
            for k in 0..i {
                q -= a.get(i, k) * a.get(k, j);
            }
            a.set(i, j, q);
        }

        // Calculate lower triangular elements for this column, tracking the pivot.
        let mut largest = 0.0f64;
        let mut largest_row = j;
        for i in j..n {
            let mut q = a.get(i, j);
            for k in 0..j {
                q -= a.get(i, k) * a.get(k, j);
            }
            a.set(i, j, q);
            let x = q.abs();
            if x >= largest {
                largest = x;
                largest_row = i;
            }
        }

        // Guard against a NaN column, which would otherwise leave `largest_row`
        // uninitialised. `runCircuit` rejects NaN matrices before factoring, so this
        // only exists to keep the function total.
        if !largest.is_finite() {
            largest_row = j;
        }

        // Pivoting.
        a.swap_rows(j, largest_row);

        // Keep track of row interchanges.
        ipvt[j] = largest_row as u32;

        // Avoid zeros.
        if a.get(j, j) == 0.0 {
            a.set(j, j, 1e-18);
        }

        if j != n - 1 {
            let mult = 1.0 / a.get(j, j);
            for i in (j + 1)..n {
                let v = a.get(i, j) * mult;
                a.set(i, j, v);
            }
        }
    }
    true
}

/// Solves `A x = b` in place using the factors produced by [`lu_factor`].
///
/// `b` is the right-hand side on input and receives the solution on output.
///
/// Port of `lu_solve`. The initial swap loop exists purely to find the first non-zero
/// right-hand-side entry so the forward substitution can start one step in.
pub fn lu_solve(a: &Matrix, ipvt: &[u32], b: &mut [f64]) {
    let n = a.size();
    if n == 0 {
        return;
    }
    debug_assert!(b.len() >= n, "b must be at least n entries");
    debug_assert!(ipvt.len() >= n, "ipvt must be at least n entries");

    // Find first nonzero b element.
    let mut i = 0usize;
    while i < n {
        let row = ipvt[i] as usize;
        let swap = b[row];
        b[row] = b[i];
        b[i] = swap;
        if swap != 0.0 {
            break;
        }
        i += 1;
    }

    // Forward substitution using the lower triangular matrix.
    let bi = i;
    i += 1;
    while i < n {
        let row = ipvt[i] as usize;
        let mut tot = b[row];
        b[row] = b[i];
        for j in bi..i {
            tot -= a.get(i, j) * b[j];
        }
        b[i] = tot;
        i += 1;
    }

    // Back-substitution using the upper triangular matrix.
    for i in (0..n).rev() {
        let mut tot = b[i];
        for j in (i + 1)..n {
            tot -= a.get(i, j) * b[j];
        }
        b[i] = tot / a.get(i, i);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Solves a dense system the long way round, purely to cross-check the LU path.
    fn solve_dense(a0: &Matrix, b0: &[f64]) -> Vec<f64> {
        let n = a0.size();
        let mut aug: Vec<Vec<f64>> = (0..n)
            .map(|i| {
                let mut row: Vec<f64> = (0..n).map(|j| a0.get(i, j)).collect();
                row.push(b0[i]);
                row
            })
            .collect();
        for col in 0..n {
            let mut piv = col;
            for r in col..n {
                if aug[r][col].abs() > aug[piv][col].abs() {
                    piv = r;
                }
            }
            aug.swap(col, piv);
            let d = aug[col][col];
            for j in col..=n {
                aug[col][j] /= d;
            }
            for r in 0..n {
                if r == col {
                    continue;
                }
                let f = aug[r][col];
                if f == 0.0 {
                    continue;
                }
                for j in col..=n {
                    aug[r][j] -= f * aug[col][j];
                }
            }
        }
        (0..n).map(|i| aug[i][n]).collect()
    }

    fn check(mut m: Matrix, rhs: Vec<f64>) {
        let n = m.size();
        let expected = solve_dense(&m, &rhs);
        let mut ipvt = vec![0u32; n];
        assert!(lu_factor(&mut m, &mut ipvt), "factorisation should succeed");
        let mut b = rhs;
        lu_solve(&m, &ipvt, &mut b);
        for i in 0..n {
            assert!(
                (b[i] - expected[i]).abs() < 1e-9 * expected[i].abs().max(1.0),
                "row {i}: got {} want {}",
                b[i],
                expected[i]
            );
        }
    }

    #[test]
    fn solves_identity() {
        let mut m = Matrix::zeros(3);
        for i in 0..3 {
            m.set(i, i, 1.0);
        }
        check(m, vec![1.0, -2.0, 3.0]);
    }

    #[test]
    fn solves_trivial_diagonal() {
        let mut m = Matrix::zeros(3);
        for i in 0..3 {
            m.set(i, i, 2.0 + i as f64);
        }
        check(m, vec![4.0, 6.0, 8.0]);
    }

    /// The voltage-divider MNA system: two 10k resistors in series across a 10V source.
    /// Mirrors `public/circuits/voltdivide.txt` reduced to one divider.
    #[test]
    fn solves_voltage_divider_mna() {
        let g = 1.0 / 10_000.0;
        let mut m = Matrix::zeros(3);
        // node 1: two resistors to ground
        m.add(0, 0, g);
        m.add(0, 1, -g);
        m.add(1, 0, -g);
        m.add(1, 1, 2.0 * g);
        m.add(1, 2, -g);
        m.add(2, 0, -g);
        m.add(2, 2, g);
        // voltage source row/column
        m.add(0, 2, 1.0);
        m.add(2, 0, 1.0);
        check(m, vec![0.0, 0.0, 10.0]);
    }

    #[test]
    fn zero_row_is_singular() {
        let mut m = Matrix::zeros(2);
        m.set(0, 0, 1.0);
        // row 1 left as all zeroes
        let mut ipvt = vec![0u32; 2];
        assert!(!lu_factor(&mut m, &mut ipvt));
    }

    #[test]
    fn empty_matrix_is_trivially_ok() {
        let mut m = Matrix::zeros(0);
        let mut ipvt: Vec<u32> = Vec::new();
        assert!(lu_factor(&mut m, &mut ipvt));
        let mut b: Vec<f64> = Vec::new();
        lu_solve(&m, &ipvt, &mut b);
    }

    #[test]
    fn swap_rows_exchanges_whole_row() {
        let mut m = Matrix::zeros(2);
        m.set(0, 0, 1.0);
        m.set(0, 1, 2.0);
        m.set(1, 0, 3.0);
        m.set(1, 1, 4.0);
        m.swap_rows(0, 1);
        assert_eq!(m.row(0), &[3.0, 4.0]);
        assert_eq!(m.row(1), &[1.0, 2.0]);
    }
}
