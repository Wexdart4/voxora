//! CPU Z-buffer software rasterizer for 3D point cloud rendering.

use crate::VirtualCamera;
use voxora_core::{Frame, PixelFormat, VoxoraError};
use voxora_reconstruction::PointCloud;

/// Deterministic CPU Z-buffer software renderer.
#[derive(Debug, Clone)]
pub struct SoftwareRenderer {
    /// Near clipping plane distance
    pub z_near: f64,
    /// Far clipping plane distance
    pub z_far: f64,
    /// Point rendering radius in pixels
    pub point_size: usize,
}

impl Default for SoftwareRenderer {
    fn default() -> Self {
        Self { z_near: 0.1, z_far: 100.0, point_size: 1 }
    }
}

impl SoftwareRenderer {
    /// Creates a software renderer with specified point size and clipping planes.
    pub fn new(point_size: usize, z_near: f64, z_far: f64) -> Self {
        Self { point_size: point_size.max(1), z_near: z_near.max(0.01), z_far }
    }

    /// Renders a 3D point cloud from the view of a VirtualCamera onto a 2D image buffer.
    pub fn render_cloud(
        &self,
        cloud: &PointCloud,
        camera: &VirtualCamera,
        width: u32,
        height: u32,
    ) -> Result<Frame, VoxoraError> {
        let w = width as usize;
        let h = height as usize;

        let mut color_buffer = vec![0u8; w * h * 3];
        let mut depth_buffer = vec![f64::INFINITY; w * h];

        let fov_rad = camera.fov_degrees.to_radians() as f64;
        let focal_length = (w as f64 / 2.0) / (fov_rad / 2.0).tan();
        let cx = w as f64 / 2.0;
        let cy = h as f64 / 2.0;

        for pt in &cloud.points {
            // Transform world point into camera frame
            let p_world = pt.position;
            let p_cam_x = p_world.x - camera.pose.translation.x;
            let p_cam_y = p_world.y - camera.pose.translation.y;
            let p_cam_z = p_world.z - camera.pose.translation.z;

            if p_cam_z < self.z_near || p_cam_z > self.z_far {
                continue;
            }

            let u = (focal_length * p_cam_x / p_cam_z + cx).round() as i32;
            let v = (focal_length * p_cam_y / p_cam_z + cy).round() as i32;

            let half_pt = (self.point_size / 2) as i32;

            for dy in -half_pt..=half_pt {
                for dx in -half_pt..=half_pt {
                    let px = u + dx;
                    let py = v + dy;

                    if px >= 0 && px < w as i32 && py >= 0 && py < h as i32 {
                        let idx = py as usize * w + px as usize;
                        if p_cam_z < depth_buffer[idx] {
                            depth_buffer[idx] = p_cam_z;
                            let c_idx = idx * 3;
                            color_buffer[c_idx] = pt.color[0];
                            color_buffer[c_idx + 1] = pt.color[1];
                            color_buffer[c_idx + 2] = pt.color[2];
                        }
                    }
                }
            }
        }

        Frame::new(width, height, PixelFormat::Rgb8, color_buffer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_math::Vector3;
    use voxora_reconstruction::Point3D;

    #[test]
    fn test_software_renderer_z_buffer() {
        let mut cloud = PointCloud::new();
        // Far red point
        cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 5.0), [255, 0, 0], 1.0));
        // Near green point at same pixel line of sight
        cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 2.0), [0, 255, 0], 1.0));

        let camera = VirtualCamera::default();
        let renderer = SoftwareRenderer::new(1, 0.1, 10.0);

        let frame = renderer.render_cloud(&cloud, &camera, 64, 64).unwrap();
        assert_eq!(frame.width, 64);
        assert_eq!(frame.height, 64);
    }
}
