//! Mathematical primitives and linear algebra for Voxora.
//!
//! Provides deterministic vectors, matrices, homogeneous transformations,
//! and numerical stability utilities.

#![warn(missing_docs)]

pub mod solvers;

pub use solvers::{least_squares_solve, sanitize_float, svd_3x3, SvdResult3x3};

/// Default numerical precision tolerance for floating-point comparisons.
pub const EPSILON: f64 = 1e-9;

/// 3D Vector representation in continuous spatial coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    /// X coordinate
    pub x: f64,
    /// Y coordinate
    pub y: f64,
    /// Z coordinate
    pub z: f64,
}

impl Vector3 {
    /// Zero vector constant.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    /// Creates a new 3D vector.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Computes the dot product between two vectors.
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Computes the cross product between two vectors.
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Computes the Euclidean norm (magnitude) of the vector.
    pub fn norm(&self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Returns a normalized unit vector, or None if the vector is zero/near-zero.
    pub fn normalize(&self) -> Option<Self> {
        let mag = self.norm();
        if mag < EPSILON {
            None
        } else {
            Some(Self { x: self.x / mag, y: self.y / mag, z: self.z / mag })
        }
    }
}

/// 3x3 Matrix representation in column-major order for camera and projection mathematics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix3x3 {
    /// Elements stored as flat row-major or column-major [f64; 9]
    pub data: [f64; 9],
}

impl Matrix3x3 {
    /// Identity matrix 3x3.
    pub const IDENTITY: Self = Self { data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] };

    /// Returns identity matrix.
    pub fn identity() -> Self {
        Self::IDENTITY
    }

    /// Creates a 3x3 matrix from a flat 9-element array in row-major order.
    pub fn from_row_major(data: [f64; 9]) -> Self {
        Self { data }
    }

    /// Returns element at (row, col).
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * 3 + col]
    }

    /// Sets element at (row, col).
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row * 3 + col] = val;
    }

    /// Computes the determinant of the 3x3 matrix.
    pub fn determinant(&self) -> f64 {
        let m = &self.data;
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6])
    }

    /// Multiplies matrix by a 3D vector.
    pub fn mul_vec(&self, v: Vector3) -> Vector3 {
        let m = &self.data;
        Vector3 {
            x: m[0] * v.x + m[1] * v.y + m[2] * v.z,
            y: m[3] * v.x + m[4] * v.y + m[5] * v.z,
            z: m[6] * v.x + m[7] * v.y + m[8] * v.z,
        }
    }

    /// Multiplies two 3x3 matrices together.
    pub fn mul_mat(&self, other: &Self) -> Self {
        let mut out = [0.0; 9];
        for r in 0..3 {
            for c in 0..3 {
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += self.get(r, k) * other.get(k, c);
                }
                out[r * 3 + c] = sum;
            }
        }
        Self::from_row_major(out)
    }

    /// Computes inverse of 3x3 matrix, or None if singular.
    pub fn invert(&self) -> Option<Self> {
        let det = self.determinant();
        if det.abs() < EPSILON {
            return None;
        }
        let inv_det = 1.0 / det;
        let m = &self.data;

        let out = [
            (m[4] * m[8] - m[5] * m[7]) * inv_det,
            (m[2] * m[7] - m[1] * m[8]) * inv_det,
            (m[1] * m[5] - m[2] * m[4]) * inv_det,
            (m[5] * m[6] - m[3] * m[8]) * inv_det,
            (m[0] * m[8] - m[2] * m[6]) * inv_det,
            (m[2] * m[3] - m[0] * m[5]) * inv_det,
            (m[3] * m[7] - m[4] * m[6]) * inv_det,
            (m[1] * m[6] - m[0] * m[7]) * inv_det,
            (m[0] * m[4] - m[1] * m[3]) * inv_det,
        ];
        Some(Self::from_row_major(out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector_dot_and_cross() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        assert_eq!(v1.dot(&v2), 0.0);

        let v3 = v1.cross(&v2);
        assert_eq!(v3, Vector3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_vector_normalize() {
        let v = Vector3::new(3.0, 0.0, 0.0);
        let norm = v.normalize().unwrap();
        assert!((norm.x - 1.0).abs() < EPSILON);
        assert_eq!(norm.y, 0.0);
        assert_eq!(norm.z, 0.0);
    }

    #[test]
    fn test_matrix_determinant_identity() {
        let mat = Matrix3x3::IDENTITY;
        assert!((mat.determinant() - 1.0).abs() < EPSILON);

        let v = Vector3::new(2.0, 3.0, 4.0);
        assert_eq!(mat.mul_vec(v), v);
    }
}
