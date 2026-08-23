//! Quaternion rotation and SE(3) Special Euclidean rigid body transformation group representations.

use voxora_math::{Matrix3x3, Vector3};

/// Unit Quaternion representing 3D rotation $q = w + x i + y j + z k$.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// Real component w
    pub w: f64,
    /// Imaginary component x
    pub x: f64,
    /// Imaginary component y
    pub y: f64,
    /// Imaginary component z
    pub z: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    /// Identity rotation quaternion $(1, 0, 0, 0)$.
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    /// Creates a quaternion.
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        let norm = (w * w + x * x + y * y + z * z).sqrt().max(1e-12);
        Self { w: w / norm, x: x / norm, y: y / norm, z: z / norm }
    }

    /// Converts a $3 \times 3$ rotation matrix into a unit Quaternion.
    pub fn from_rotation_matrix(m: &Matrix3x3) -> Self {
        let trace = m.get(0, 0) + m.get(1, 1) + m.get(2, 2);
        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            let w = 0.25 / s;
            let x = (m.get(2, 1) - m.get(1, 2)) * s;
            let y = (m.get(0, 2) - m.get(2, 0)) * s;
            let z = (m.get(1, 0) - m.get(0, 1)) * s;
            Self::new(w, x, y, z)
        } else if m.get(0, 0) > m.get(1, 1) && m.get(0, 0) > m.get(2, 2) {
            let s = 2.0 * (1.0 + m.get(0, 0) - m.get(1, 1) - m.get(2, 2)).sqrt();
            let w = (m.get(2, 1) - m.get(1, 2)) / s;
            let x = 0.25 * s;
            let y = (m.get(0, 1) + m.get(1, 0)) / s;
            let z = (m.get(0, 2) + m.get(2, 0)) / s;
            Self::new(w, x, y, z)
        } else if m.get(1, 1) > m.get(2, 2) {
            let s = 2.0 * (1.0 + m.get(1, 1) - m.get(0, 0) - m.get(2, 2)).sqrt();
            let w = (m.get(0, 2) - m.get(2, 0)) / s;
            let x = (m.get(0, 1) + m.get(1, 0)) / s;
            let y = 0.25 * s;
            let z = (m.get(1, 2) + m.get(2, 1)) / s;
            Self::new(w, x, y, z)
        } else {
            let s = 2.0 * (1.0 + m.get(2, 2) - m.get(0, 0) - m.get(1, 1)).sqrt();
            let w = (m.get(1, 0) - m.get(0, 1)) / s;
            let x = (m.get(0, 2) + m.get(2, 0)) / s;
            let y = (m.get(1, 2) + m.get(2, 1)) / s;
            let z = 0.25 * s;
            Self::new(w, x, y, z)
        }
    }

    /// Converts quaternion into a $3 \times 3$ rotation matrix.
    pub fn to_rotation_matrix(&self) -> Matrix3x3 {
        let w = self.w;
        let x = self.x;
        let y = self.y;
        let z = self.z;

        Matrix3x3::from_row_major([
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ])
    }

    /// Spherical Linear Interpolation (SLERP) between two quaternions for factor $t \in [0, 1]$.
    pub fn slerp(q1: &Self, q2: &Self, t: f64) -> Self {
        let mut dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
        let mut q2_adj = *q2;

        if dot < 0.0 {
            dot = -dot;
            q2_adj = Self::new(-q2.w, -q2.x, -q2.y, -q2.z);
        }

        if dot > 0.9995 {
            // Linear interpolation for tiny angles
            let w = q1.w + t * (q2_adj.w - q1.w);
            let x = q1.x + t * (q2_adj.x - q1.x);
            let y = q1.y + t * (q2_adj.y - q1.y);
            let z = q1.z + t * (q2_adj.z - q1.z);
            return Self::new(w, x, y, z);
        }

        let theta_0 = dot.acos();
        let theta = theta_0 * t;
        let sin_theta = theta.sin();
        let sin_theta_0 = theta_0.sin();

        let s0 = (theta_0 - theta).sin() / sin_theta_0;
        let s1 = sin_theta / sin_theta_0;

        Self::new(
            s0 * q1.w + s1 * q2_adj.w,
            s0 * q1.x + s1 * q2_adj.x,
            s0 * q1.y + s1 * q2_adj.y,
            s0 * q1.z + s1 * q2_adj.z,
        )
    }
}

/// Special Euclidean Group $SE(3)$ rigid body transformation composed of rotation $R$ and translation $\mathbf{t}$.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransformSE3 {
    /// Rotation matrix $R \in SO(3)$
    pub rotation: Matrix3x3,
    /// Translation vector $\mathbf{t} \in \mathbb{R}^3$
    pub translation: Vector3,
}

impl Default for TransformSE3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl TransformSE3 {
    /// Identity rigid transformation.
    pub const IDENTITY: Self = Self { rotation: Matrix3x3::IDENTITY, translation: Vector3::ZERO };

    /// Creates an SE(3) transformation from rotation matrix and translation vector.
    pub fn new(rotation: Matrix3x3, translation: Vector3) -> Self {
        Self { rotation, translation }
    }

    /// Creates an SE(3) transformation from Quaternion rotation and translation vector.
    pub fn from_quaternion_and_translation(q: Quaternion, translation: Vector3) -> Self {
        Self { rotation: q.to_rotation_matrix(), translation }
    }

    /// Applies SE(3) transformation to a 3D point $P' = R P + \mathbf{t}$.
    pub fn transform_point(&self, p: Vector3) -> Vector3 {
        let r_p = self.rotation.mul_vec(p);
        Vector3::new(
            r_p.x + self.translation.x,
            r_p.y + self.translation.y,
            r_p.z + self.translation.z,
        )
    }

    /// Composes this transformation with another $T_{combined} = T_1 \cdot T_2$.
    pub fn compose(&self, other: &Self) -> Self {
        let r_combined = self.rotation.mul_mat(&other.rotation);
        let r_t = self.rotation.mul_vec(other.translation);
        let t_combined = Vector3::new(
            self.translation.x + r_t.x,
            self.translation.y + r_t.y,
            self.translation.z + r_t.z,
        );
        Self::new(r_combined, t_combined)
    }

    /// Computes the inverse transformation $T^{-1} = (R^T, -R^T \mathbf{t})$.
    pub fn inverse(&self) -> Self {
        let r_inv = Matrix3x3::from_row_major([
            self.rotation.get(0, 0),
            self.rotation.get(1, 0),
            self.rotation.get(2, 0),
            self.rotation.get(0, 1),
            self.rotation.get(1, 1),
            self.rotation.get(2, 1),
            self.rotation.get(0, 2),
            self.rotation.get(1, 2),
            self.rotation.get(2, 2),
        ]);
        let neg_t = Vector3::new(-self.translation.x, -self.translation.y, -self.translation.z);
        let t_inv = r_inv.mul_vec(neg_t);
        Self::new(r_inv, t_inv)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quaternion_slerp() {
        let q1 = Quaternion::IDENTITY;
        let frac = std::f64::consts::FRAC_1_SQRT_2;
        let q2 = Quaternion::new(frac, 0.0, frac, 0.0);
        let q_mid = Quaternion::slerp(&q1, &q2, 0.5);
        assert!((q_mid.w - 1.0).abs() > 1e-4);
    }

    #[test]
    fn test_transform_se3_inverse() {
        let t = TransformSE3::new(Matrix3x3::IDENTITY, Vector3::new(1.0, 2.0, 3.0));
        let t_inv = t.inverse();
        let p = Vector3::new(5.0, 5.0, 5.0);
        let p_trans = t.transform_point(p);
        let p_orig = t_inv.transform_point(p_trans);
        assert!((p_orig.x - p.x).abs() < 1e-5);
        assert!((p_orig.y - p.y).abs() < 1e-5);
        assert!((p_orig.z - p.z).abs() < 1e-5);
    }
}
