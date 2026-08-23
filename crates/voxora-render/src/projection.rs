//! Perspective spatial 2D-to-3D frame pixel projection and stereo camera mapping.

use voxora_core::{Frame, VoxoraError};
use voxora_geometry::{CameraIntrinsics, TransformSE3};
use voxora_math::Vector3;
use voxora_reconstruction::{Point3D, PointCloud};

/// Perspective spatial video projector mapping 2D frame pixels and depth into 3D world coordinates.
#[derive(Debug, Clone)]
pub struct SpatialProjector {
    /// Camera intrinsics
    pub intrinsics: CameraIntrinsics,
    /// World pose transformation SE(3)
    pub pose: TransformSE3,
}

impl SpatialProjector {
    /// Creates a new spatial projector.
    pub fn new(intrinsics: CameraIntrinsics, pose: TransformSE3) -> Self {
        Self { intrinsics, pose }
    }

    /// Maps pixel coordinate $(u, v)$ with depth $Z$ into a 3D point in world coordinates.
    pub fn project_pixel_to_3d(&self, u: f64, v: f64, depth: f64, color: [u8; 3]) -> Point3D {
        let (norm_x, norm_y) = self.intrinsics.pixel_to_normalized(u, v);
        let point_cam = Vector3::new(norm_x * depth, norm_y * depth, depth);
        let point_world = self.pose.transform_point(point_cam);
        Point3D::new(point_world, color, 1.0)
    }

    /// Projects an entire RGB video frame with dense depth map into a 3D Point Cloud.
    pub fn project_frame(
        &self,
        frame: &Frame,
        depth_map: &[f32],
        stride: usize,
    ) -> Result<PointCloud, VoxoraError> {
        let width = frame.width as usize;
        let height = frame.height as usize;
        let mut cloud = PointCloud::new();

        let step = stride.max(1);

        for y in (0..height).step_by(step) {
            for x in (0..width).step_by(step) {
                let idx = y * width + x;
                let depth = depth_map[idx] as f64;
                if depth > 0.1 {
                    let pixel_idx = (y * width + x) * 3;
                    let color = if pixel_idx + 2 < frame.data.len() {
                        [
                            frame.data[pixel_idx],
                            frame.data[pixel_idx + 1],
                            frame.data[pixel_idx + 2],
                        ]
                    } else {
                        [128, 128, 128]
                    };

                    let pt = self.project_pixel_to_3d(x as f64, y as f64, depth, color);
                    cloud.push(pt);
                }
            }
        }

        Ok(cloud)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_project_pixel_to_3d() {
        let intr = CameraIntrinsics::from_fov(60.0, 640, 480);
        let proj = SpatialProjector::new(intr, TransformSE3::IDENTITY);
        let pt = proj.project_pixel_to_3d(320.0, 240.0, 5.0, [255, 0, 0]);
        assert!((pt.position.z - 5.0).abs() < 1e-4);
    }
}
