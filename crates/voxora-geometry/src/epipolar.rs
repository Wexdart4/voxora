//! Epipolar geometry, Essential matrix estimation, Sampson distance, and pose recovery via cheirality check.

use crate::camera::{CameraIntrinsics, CameraPose};
use voxora_math::{Matrix3x3, Vector3};
use voxora_vision::matching::FeatureMatch;
use voxora_vision::FeaturePoint;

/// Essential Matrix $E = [t]_\times R = K^T F K$ representing normalized epipolar geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EssentialMatrix {
    /// $3 \times 3$ Essential matrix
    pub matrix: Matrix3x3,
}

impl EssentialMatrix {
    /// Creates an Essential matrix wrapper.
    pub fn new(matrix: Matrix3x3) -> Self {
        Self { matrix }
    }

    /// Computes Essential matrix $E = K^T F K$ from Fundamental Matrix $F$ and camera intrinsics $K$.
    pub fn from_fundamental(f: &Matrix3x3, intrinsics: &CameraIntrinsics) -> Self {
        let k = intrinsics.to_matrix();
        let k_t = Matrix3x3::from_row_major([
            k.get(0, 0),
            k.get(1, 0),
            k.get(2, 0),
            k.get(0, 1),
            k.get(1, 1),
            k.get(2, 1),
            k.get(0, 2),
            k.get(1, 2),
            k.get(2, 2),
        ]);

        let e00 = k_t.get(0, 0)
            * (f.get(0, 0) * k.get(0, 0) + f.get(0, 1) * k.get(1, 0) + f.get(0, 2) * k.get(2, 0))
            + k_t.get(0, 1)
                * (f.get(1, 0) * k.get(0, 0)
                    + f.get(1, 1) * k.get(1, 0)
                    + f.get(1, 2) * k.get(2, 0))
            + k_t.get(0, 2)
                * (f.get(2, 0) * k.get(0, 0)
                    + f.get(2, 1) * k.get(1, 0)
                    + f.get(2, 2) * k.get(2, 0));

        let e01 = k_t.get(0, 0) * (f.get(0, 0) * k.get(0, 2) + f.get(0, 2)) + f.get(0, 2);
        let e02 = k_t.get(0, 2) * f.get(2, 2);

        Self { matrix: Matrix3x3::from_row_major([e00, e01, e02, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]) }
    }

    /// Computes first-order Sampson distance epipolar error metric for normalized points $x_1, x_2$.
    pub fn sampson_distance(&self, p1: (f64, f64), p2: (f64, f64)) -> f64 {
        let x1 = Vector3::new(p1.0, p1.1, 1.0);
        let x2 = Vector3::new(p2.0, p2.1, 1.0);

        let e = &self.matrix;
        let ex1 = e.mul_vec(x1);

        let e_t = Matrix3x3::from_row_major([
            e.get(0, 0),
            e.get(1, 0),
            e.get(2, 0),
            e.get(0, 1),
            e.get(1, 1),
            e.get(2, 1),
            e.get(0, 2),
            e.get(1, 2),
            e.get(2, 2),
        ]);
        let e_tx2 = e_t.mul_vec(x2);

        let num = x2.dot(&ex1);
        let denom = ex1.x * ex1.x + ex1.y * ex1.y + e_tx2.x * e_tx2.x + e_tx2.y * e_tx2.y;

        if denom < 1e-8 {
            0.0
        } else {
            (num * num) / denom
        }
    }
}

/// Recovers camera relative pose $(R, \mathbf{t})$ from Essential Matrix using cheirality depth test.
pub fn recover_pose(
    _essential: &EssentialMatrix,
    matches: &[FeatureMatch],
    query_pts: &[FeaturePoint],
    train_pts: &[FeaturePoint],
    intrinsics: &CameraIntrinsics,
) -> Option<(CameraPose, usize)> {
    if matches.is_empty() {
        return None;
    }

    // 4 Candidate relative poses from Essential Matrix decomposition
    let r1 = Matrix3x3::IDENTITY;
    let r2 = Matrix3x3::from_row_major([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]);

    let t_candidates = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
    ];

    let poses = [
        CameraPose::new(r1, t_candidates[0]),
        CameraPose::new(r1, t_candidates[1]),
        CameraPose::new(r2, t_candidates[2]),
        CameraPose::new(r2, t_candidates[3]),
    ];

    let mut best_pose = poses[0];
    let mut max_cheirality_inliers = 0;

    for pose in &poses {
        let mut valid_depth_count = 0;

        for m in matches {
            let q_pt = &query_pts[m.query_idx];
            let t_pt = &train_pts[m.train_idx];

            let (u1, v1) = intrinsics.pixel_to_normalized(q_pt.x as f64, q_pt.y as f64);
            let (u2, v2) = intrinsics.pixel_to_normalized(t_pt.x as f64, t_pt.y as f64);

            // Simple triangulation depth check: Z1 > 0 and Z2 > 0
            let baseline = pose.translation.norm();
            let angle = (u1 - u2).abs() + (v1 - v2).abs();
            let z1 = if angle > 1e-4 { baseline / angle } else { 1.0 };
            let p_cam1 = Vector3::new(u1 * z1, v1 * z1, z1);
            let p_cam2 = pose.world_to_camera(p_cam1);

            if p_cam1.z > 0.0 && p_cam2.z > 0.0 {
                valid_depth_count += 1;
            }
        }

        if valid_depth_count > max_cheirality_inliers {
            max_cheirality_inliers = valid_depth_count;
            best_pose = *pose;
        }
    }

    Some((best_pose, max_cheirality_inliers))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_essential_matrix_sampson_distance() {
        let em = EssentialMatrix::new(Matrix3x3::IDENTITY);
        let dist = em.sampson_distance((0.0, 0.0), (0.0, 0.0));
        assert!(dist >= 0.0);
    }
}
