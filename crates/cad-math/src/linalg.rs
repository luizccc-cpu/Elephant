use glam::DVec3;

/// Solve a dense `n x n` linear system `A x = b` via Gaussian elimination with partial pivoting.
pub fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = a.len();
    if n == 0 || b.len() != n || a.iter().any(|row| row.len() != n) {
        return None;
    }

    let mut mat = a.to_vec();
    let mut rhs = b.to_vec();

    for col in 0..n {
        let mut pivot_row = col;
        let mut max_val = mat[col][col].abs();
        for (r, row) in mat.iter().enumerate().skip(col + 1) {
            let v = row[col].abs();
            if v > max_val {
                max_val = v;
                pivot_row = r;
            }
        }
        if !max_val.is_finite() || max_val < 1e-14 {
            return None;
        }
        if pivot_row != col {
            mat.swap(col, pivot_row);
            rhs.swap(col, pivot_row);
        }

        let pivot = mat[col][col];
        for r in (col + 1)..n {
            let factor = mat[r][col] / pivot;
            if factor == 0.0 {
                continue;
            }
            mat[r][col] = 0.0;
            for c in (col + 1)..n {
                mat[r][c] -= factor * mat[col][c];
            }
            rhs[r] -= factor * rhs[col];
        }
    }

    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let mut sum = rhs[r];
        for c in (r + 1)..n {
            sum -= mat[r][c] * x[c];
        }
        x[r] = sum / mat[r][r];
    }
    Some(x)
}

/// Solve `A X = B` for 3D vector columns `(x, y, z)` simultaneously via partial pivoting.
pub fn solve_linear_system_vec3(a: &[Vec<f64>], rhs: &[DVec3]) -> Option<Vec<DVec3>> {
    let n = a.len();
    if n == 0 || rhs.len() != n || a.iter().any(|row| row.len() != n) {
        return None;
    }

    let mut mat = a.to_vec();
    let mut b = rhs.to_vec();

    for col in 0..n {
        let mut pivot_row = col;
        let mut max_val = mat[col][col].abs();
        for (r, row) in mat.iter().enumerate().skip(col + 1) {
            let v = row[col].abs();
            if v > max_val {
                max_val = v;
                pivot_row = r;
            }
        }
        if !max_val.is_finite() || max_val < 1e-14 {
            return None;
        }
        if pivot_row != col {
            mat.swap(col, pivot_row);
            b.swap(col, pivot_row);
        }

        let pivot = mat[col][col];
        let b_col = b[col];
        for r in (col + 1)..n {
            let factor = mat[r][col] / pivot;
            if factor == 0.0 {
                continue;
            }
            mat[r][col] = 0.0;
            for c in (col + 1)..n {
                mat[r][c] -= factor * mat[col][c];
            }
            b[r] -= b_col * factor;
        }
    }

    let mut x = vec![DVec3::ZERO; n];
    for r in (0..n).rev() {
        let mut sum = b[r];
        for c in (r + 1)..n {
            sum -= x[c] * mat[r][c];
        }
        x[r] = sum / mat[r][r];
    }
    Some(x)
}

/// Solve regularized normal equations `(A^T A + ridge * I) X = A^T B` for 3D vectors.
pub fn solve_least_squares_vec3(
    design: &[Vec<f64>],
    rhs: &[DVec3],
    ridge: f64,
) -> Option<Vec<DVec3>> {
    let m = design.len();
    if m == 0 || rhs.len() != m {
        return None;
    }
    let k = design[0].len();
    if k == 0 || design.iter().any(|row| row.len() != k) {
        return None;
    }

    let mut ata = vec![vec![0.0; k]; k];
    let mut atb = vec![DVec3::ZERO; k];

    for (row, &b_val) in design.iter().zip(rhs.iter()) {
        for i in 0..k {
            let ri = row[i];
            if ri == 0.0 {
                continue;
            }
            atb[i] += b_val * ri;
            for j in i..k {
                ata[i][j] += ri * row[j];
            }
        }
    }
    for i in 0..k {
        ata[i][i] += ridge;
        for j in 0..i {
            ata[i][j] = ata[j][i];
        }
    }

    solve_linear_system_vec3(&ata, &atb)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_linear_system_and_vec3_and_rejects_singular() {
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let b = vec![5.0, 10.0];
        let x = solve_linear_system(&a, &b).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-12);
        assert!((x[1] - 3.0).abs() < 1e-12);

        let rhs = vec![DVec3::new(5.0, 4.0, 0.0), DVec3::new(10.0, 7.0, 5.0)];
        let xv = solve_linear_system_vec3(&a, &rhs).unwrap();
        assert!((xv[0] - DVec3::new(1.0, 1.0, -1.0)).length() < 1e-12);
        assert!((xv[1] - DVec3::new(3.0, 2.0, 2.0)).length() < 1e-12);

        let singular = vec![vec![1.0, 2.0], vec![2.0, 4.0]];
        assert!(solve_linear_system(&singular, &[1.0, 2.0]).is_none());
    }

    #[test]
    fn solves_least_squares_line_fit() {
        // y = 2 * t + 1 at t = 0, 1, 2, 3
        let design = vec![
            vec![1.0, 0.0],
            vec![1.0, 1.0],
            vec![1.0, 2.0],
            vec![1.0, 3.0],
        ];
        let rhs = vec![
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(1.0, 3.0, 0.0),
            DVec3::new(2.0, 5.0, 0.0),
            DVec3::new(3.0, 7.0, 0.0),
        ];
        let sol = solve_least_squares_vec3(&design, &rhs, 0.0).unwrap();
        assert!((sol[0] - DVec3::new(0.0, 1.0, 0.0)).length() < 1e-11);
        assert!((sol[1] - DVec3::new(1.0, 2.0, 0.0)).length() < 1e-11);
    }
}
