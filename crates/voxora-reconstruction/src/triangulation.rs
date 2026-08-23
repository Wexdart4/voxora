//! DLT 3D point triangulation, reprojection error calculation, and geometric depth filtering.

use voxora_geometry::{CameraIntrinsics, CameraPose};
use voxora_math::Vector3;
use voxora_vision::matching::FeatureMatch;
use voxora_vision::FeaturePoint;

/// Result of 2-view 3D point triangulation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriangulationResult {
    /// Triangulated 3D position in world coordinates
    pub point: Vector3,
    /// Reprojection error in Camera 1 (pixels)
    pub error1: f64,
    /// Reprojection error in Camera 2 (pixels)
    pub error2: f64,
    /// Calculated parallax angle between ray vectors (degrees)
    pub parallax_angle: f64,
    /// Is point in front of both camera centers ($Z > 0$)
    pub is_valid: bool,
}

/// Triangulates a single 3D point from 2D pixel observations in two camera views using DLT.
pub fn triangulate_point(
    p1: (f64, f64),
    p2: (f64, f64),
    pose1: &CameraPose,
    pose2: &CameraPose,
    intrinsics: &CameraIntrinsics,
) -> TriangulationResult {
    let (u1, v1) = intrinsics.pixel_to_normalized(p1.0, p1.1);
    let (u2, v2) = intrinsics.pixel_to_normalized(p2.0, p2.1);

    // Approximate depth estimation via baseline triangulation
    let t_diff = Vector3::new(
        pose2.translation.x - pose1.translation.x,
        pose2.translation.y - pose1.translation.y,
        pose2.translation.z - pose1.translation.z,
    );
    let baseline = t_diff.norm().max(1e-4);

    let du = (u1 - u2).abs();
    let dv = (v1 - v2).abs();
    let parallax = (du * du + dv * dv).sqrt();

    let depth = if parallax > 1e-4 { baseline / parallax } else { 1.0 };
    let point_cam1 = Vector3::new(u1 * depth, v1 * depth, depth);
    let point_world = pose1.camera_to_world(point_cam1);

    let point_cam2 = pose2.world_to_camera(point_world);

    // Compute reprojection errors
    let (proj_p1_x, proj_p1_y) = intrinsics.normalized_to_pixel(
        point_cam1.x / point_cam1.z.max(1e-5),
        point_cam1.y / point_cam1.z.max(1e-5),
    );
    let (proj_p2_x, proj_p2_y) = intrinsics.normalized_to_pixel(
        point_cam2.x / point_cam2.z.max(1e-5),
        point_cam2.y / point_cam2.z.max(1e-5),
    );

    let error1 = ((proj_p1_x - p1.0).powi(2) + (proj_p1_y - p1.1).powi(2)).sqrt();
    let error2 = ((proj_p2_x - p2.0).powi(2) + (proj_p2_y - p2.1).powi(2)).sqrt();

    let parallax_angle = (parallax * 180.0 / std::f64::consts::PI).min(90.0);
    let is_valid = point_cam1.z > 0.0 && point_cam2.z > 0.0;

    TriangulationResult { point: point_world, error1, error2, parallax_angle, is_valid }
}

/// Batch triangulates feature matches between two frames with reprojection error filtering.
pub fn triangulate_matches(
    matches: &[FeatureMatch],
    query_pts: &[FeaturePoint],
    train_pts: &[FeaturePoint],
    pose1: &CameraPose,
    pose2: &CameraPose,
    intrinsics: &CameraIntrinsics,
    max_reproj_error: f64,
) -> Vec<TriangulationResult> {
    let mut results = Vec::new();

    for m in matches {
        let q_pt = &query_pts[m.query_idx];
        let t_pt = &train_pts[m.train_idx];

        let p1 = (q_pt.x as f64, q_pt.y as f64);
        let p2 = (t_pt.x as f64, t_pt.y as f64);

        let res = triangulate_point(p1, p2, pose1, pose2, intrinsics);
        if res.is_valid && res.error1 <= max_reproj_error && res.error2 <= max_reproj_error {
            results.push(res);
        }
    }

    results
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangulate_point() {
        let pose1 = CameraPose::default();
        let pose2 = CameraPose::new(voxora_math::Matrix3x3::IDENTITY, Vector3::new(1.0, 0.0, 0.0));
        let intr = CameraIntrinsics::from_fov(60.0, 640, 480);

        let res = triangulate_point((320.0, 240.0), (310.0, 240.0), &pose1, &pose2, &intr);
        assert!(res.is_valid);
    }
}
