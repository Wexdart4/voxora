//! Stereo camera rig rendering and Side-by-Side / Top-Bottom video frame composition.

use crate::software::SoftwareRenderer;
use crate::VirtualCamera;
use voxora_core::{Frame, PixelFormat, VoxoraError};
use voxora_reconstruction::PointCloud;

/// Layout format options for stereo frame composition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StereoLayout {
    /// Horizontal Side-by-Side layout (Width = 2 * W, Height = H)
    SideBySide,
    /// Vertical Top-Bottom layout (Width = W, Height = 2 * H)
    TopBottom,
}

/// Stereo virtual camera rig for rendering binocular 3D perspectives.
#[derive(Debug, Clone)]
pub struct StereoCameraRig {
    /// Center reference camera
    pub center_camera: VirtualCamera,
    /// Interocular distance (baseline) between left and right virtual cameras
    pub interocular_distance: f64,
}

impl StereoCameraRig {
    /// Creates a stereo camera rig with specified center camera and baseline distance.
    pub fn new(center_camera: VirtualCamera, interocular_distance: f64) -> Self {
        Self { center_camera, interocular_distance }
    }

    /// Computes the Left-Eye Virtual Camera.
    pub fn left_camera(&self) -> VirtualCamera {
        let mut cam = self.center_camera.clone();
        cam.pose.translation.x -= self.interocular_distance / 2.0;
        cam
    }

    /// Computes the Right-Eye Virtual Camera.
    pub fn right_camera(&self) -> VirtualCamera {
        let mut cam = self.center_camera.clone();
        cam.pose.translation.x += self.interocular_distance / 2.0;
        cam
    }
}

/// Binocular stereo frame composer producing Side-by-Side or Top-Bottom stereo frames.
#[derive(Debug, Clone, Default)]
pub struct StereoFrameComposer {
    /// Software renderer instance
    pub renderer: SoftwareRenderer,
}

impl StereoFrameComposer {
    /// Creates a stereo frame composer with custom software renderer settings.
    pub fn new(renderer: SoftwareRenderer) -> Self {
        Self { renderer }
    }

    /// Renders left-eye and right-eye views of a point cloud and composes them into a single stereo frame.
    pub fn render_stereo_frame(
        &self,
        cloud: &PointCloud,
        rig: &StereoCameraRig,
        eye_width: u32,
        eye_height: u32,
        layout: StereoLayout,
    ) -> Result<Frame, VoxoraError> {
        let left_frame =
            self.renderer.render_cloud(cloud, &rig.left_camera(), eye_width, eye_height)?;
        let right_frame =
            self.renderer.render_cloud(cloud, &rig.right_camera(), eye_width, eye_height)?;

        match layout {
            StereoLayout::SideBySide => {
                let out_w = eye_width * 2;
                let out_h = eye_height;
                let mut out_data = vec![0u8; (out_w * out_h * 3) as usize];

                let w_bytes = (eye_width as usize) * 3;
                let out_w_bytes = (out_w as usize) * 3;

                for y in 0..eye_height as usize {
                    let left_src_idx = y * w_bytes;
                    let left_dst_idx = y * out_w_bytes;
                    out_data[left_dst_idx..left_dst_idx + w_bytes]
                        .copy_from_slice(&left_frame.data[left_src_idx..left_src_idx + w_bytes]);

                    let right_src_idx = y * w_bytes;
                    let right_dst_idx = y * out_w_bytes + w_bytes;
                    out_data[right_dst_idx..right_dst_idx + w_bytes]
                        .copy_from_slice(&right_frame.data[right_src_idx..right_src_idx + w_bytes]);
                }

                Frame::new(out_w, out_h, PixelFormat::Rgb8, out_data)
            }
            StereoLayout::TopBottom => {
                let out_w = eye_width;
                let out_h = eye_height * 2;
                let mut out_data = Vec::with_capacity((out_w * out_h * 3) as usize);

                out_data.extend_from_slice(&left_frame.data);
                out_data.extend_from_slice(&right_frame.data);

                Frame::new(out_w, out_h, PixelFormat::Rgb8, out_data)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_math::Vector3;
    use voxora_reconstruction::Point3D;

    #[test]
    fn test_stereo_frame_composer_sbs() {
        let mut cloud = PointCloud::new();
        cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 3.0), [255, 255, 255], 1.0));

        let rig = StereoCameraRig::new(VirtualCamera::default(), 0.065);
        let composer = StereoFrameComposer::default();

        let sbs_frame =
            composer.render_stereo_frame(&cloud, &rig, 32, 32, StereoLayout::SideBySide).unwrap();
        assert_eq!(sbs_frame.width, 64);
        assert_eq!(sbs_frame.height, 32);
    }
}
