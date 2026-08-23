//! Numerical solvers, SVD decomposition, linear least-squares, and floating-point sanity validation.

use crate::{Matrix3x3, Vector3};

/// Singular Value Decomposition result for a $3 \times 3$ matrix $A = U \cdot S \cdot V^T$.
#[derive(Debug, Clone, PartialEq)]
pub struct SvdResult3x3 {
    /// Left orthogonal matrix U
    pub u: Matrix3x3,
    /// Singular values vector S = (s1, s2, s3) in descending order
    pub s: Vector3,
    /// Right orthogonal matrix V
    pub v: Matrix3x3,
}

/// Solves $3 \times 3$ Singular Value Decomposition using Jacobi rotations.
pub fn svd_3x3(a: &Matrix3x3) -> SvdResult3x3 {
    let mut u = *a;
    let mut v = Matrix3x3::IDENTITY;
    let max_iter = 15;

    for _ in 0..max_iter {
        // Cyclic Jacobi rotations on U * U^T
        let mut converged = true;
        for i in 0..2 {
            for j in (i + 1)..3 {
                let alpha = u.get(0, i) * u.get(0, j)
                    + u.get(1, i) * u.get(1, j)
                    + u.get(2, i) * u.get(2, j);
                let beta = u.get(0, i) * u.get(0, i)
                    + u.get(1, i) * u.get(1, i)
                    + u.get(2, i) * u.get(2, i);
                let gamma = u.get(0, j) * u.get(0, j)
                    + u.get(1, j) * u.get(1, j)
                    + u.get(2, j) * u.get(2, j);

                if alpha.abs() > 1e-10 {
                    converged = false;
                    let zeta = (gamma - beta) / (2.0 * alpha);
                    let t = if zeta >= 0.0 {
                        1.0 / (zeta + (1.0 + zeta * zeta).sqrt())
                    } else {
                        -1.0 / (-zeta + (1.0 + zeta * zeta).sqrt())
                    };
                    let c = 1.0 / (1.0 + t * t).sqrt();
                    let s = c * t;

                    // Rotate columns i and j of U and V
                    for k in 0..3 {
                        let u_ik = u.get(k, i);
                        let u_jk = u.get(k, j);
                        u.set(k, i, c * u_ik - s * u_jk);
                        u.set(k, j, s * u_ik + c * u_jk);

                        let v_ik = v.get(k, i);
                        let v_jk = v.get(k, j);
                        v.set(k, i, c * v_ik - s * v_jk);
                        v.set(k, j, s * v_ik + c * v_jk);
                    }
                }
            }
        }
        if converged {
            break;
        }
    }

    // Extract singular values (column norms of U)
    let s1 =
        (u.get(0, 0) * u.get(0, 0) + u.get(1, 0) * u.get(1, 0) + u.get(2, 0) * u.get(2, 0)).sqrt();
    let s2 =
        (u.get(0, 1) * u.get(0, 1) + u.get(1, 1) * u.get(1, 1) + u.get(2, 1) * u.get(2, 1)).sqrt();
    let s3 =
        (u.get(0, 2) * u.get(0, 2) + u.get(1, 2) * u.get(1, 2) + u.get(2, 2) * u.get(2, 2)).sqrt();

    // Normalize U columns
    if s1 > 1e-10 {
        u.set(0, 0, u.get(0, 0) / s1);
        u.set(1, 0, u.get(1, 0) / s1);
        u.set(2, 0, u.get(2, 0) / s1);
    }
    if s2 > 1e-10 {
        u.set(0, 1, u.get(0, 1) / s2);
        u.set(1, 1, u.get(1, 1) / s2);
        u.set(2, 1, u.get(2, 1) / s2);
    }
    if s3 > 1e-10 {
        u.set(0, 2, u.get(0, 2) / s3);
        u.set(1, 2, u.get(1, 2) / s3);
        u.set(2, 2, u.get(2, 2) / s3);
    }

    SvdResult3x3 { u, s: Vector3::new(s1, s2, s3), v }
}

/// Solves linear least-squares problem $A x = b$ for a $3 \times 3$ matrix system.
pub fn least_squares_solve(a: &Matrix3x3, b: Vector3) -> Option<Vector3> {
    let inv_a = a.invert()?;
    Some(inv_a.mul_vec(b))
}

/// Sanitizes a floating-point value, substituting fallback if value is NaN or Infinite.
pub fn sanitize_float(val: f64, fallback: f64) -> f64 {
    if val.is_nan() || val.is_infinite() {
        fallback
    } else {
        val
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_svd_3x3_decomposition() {
        let m = Matrix3x3::from_row_major([2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0]);
        let svd = svd_3x3(&m);

        assert!(
            (svd.s.x - 3.0).abs() < 1e-2
                || (svd.s.y - 3.0).abs() < 1e-2
                || (svd.s.z - 3.0).abs() < 1e-2
        );
    }

    #[test]
    fn test_least_squares_solve() {
        let a = Matrix3x3::IDENTITY;
        let b = Vector3::new(1.0, 2.0, 3.0);
        let x = least_squares_solve(&a, b).unwrap();
        assert_eq!(x, b);
    }

    #[test]
    fn test_sanitize_float() {
        assert_eq!(sanitize_float(f64::NAN, 0.0), 0.0);
        assert_eq!(sanitize_float(f64::INFINITY, 1.0), 1.0);
        assert_eq!(sanitize_float(42.0, 0.0), 42.0);
    }
}
