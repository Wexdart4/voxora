//! Camera intrinsic model, radial/tangential lens distortion correction, and camera pose representations.

use voxora_core::{Frame, PixelFormat, VoxoraError};
use voxora_math::{Matrix3x3, Vector3};

/// Camera intrinsic parameters defining perspective projection geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraIntrinsics {
    /// Focal length along X axis (in pixels)
    pub fx: f64,
    /// Focal length along Y axis (in pixels)
    pub fy: f64,
    /// Principal point X coordinate (optical center, in pixels)
    pub cx: f64,
    /// Principal point Y coordinate (optical center, in pixels)
    pub cy: f64,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
}

impl CameraIntrinsics {
    /// Creates camera intrinsics from focal length and optical center.
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, width: u32, height: u32) -> Self {
        Self { fx, fy, cx, cy, width, height }
    }

    /// Default pinhole camera model based on frame dimensions and horizontal Field of View (in degrees).
    pub fn from_fov(fov_degrees: f64, width: u32, height: u32) -> Self {
        let fov_rad = fov_degrees.to_radians();
        let fx = (width as f64) / (2.0 * (fov_rad / 2.0).tan());
        let fy = fx;
        let cx = width as f64 / 2.0;
        let cy = height as f64 / 2.0;
        Self { fx, fy, cx, cy, width, height }
    }

    /// Converts intrinsics into a $3 \times 3$ intrinsic matrix $K$.
    pub fn to_matrix(&self) -> Matrix3x3 {
        Matrix3x3::from_row_major([self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0])
    }

    /// Projects 2D pixel coordinates $(x, y)$ to normalized camera coordinates $(u, v)$.
    pub fn pixel_to_normalized(&self, px: f64, py: f64) -> (f64, f64) {
        let u = (px - self.cx) / self.fx;
        let v = (py - self.cy) / self.fy;
        (u, v)
    }

    /// Projects normalized camera coordinates $(u, v)$ to 2D pixel coordinates $(x, y)$.
    pub fn normalized_to_pixel(&self, u: f64, v: f64) -> (f64, f64) {
        let px = u * self.fx + self.cx;
        let py = v * self.fy + self.cy;
        (px, py)
    }
}

/// Radial ($k_1, k_2, k_3$) and Tangential ($p_1, p_2$) lens distortion parameters.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LensDistortion {
    /// Radial distortion coefficient k1
    pub k1: f64,
    /// Radial distortion coefficient k2
    pub k2: f64,
    /// Radial distortion coefficient k3
    pub k3: f64,
    /// Tangential distortion coefficient p1
    pub p1: f64,
    /// Tangential distortion coefficient p2
    pub p2: f64,
}

impl LensDistortion {
    /// Creates a lens distortion parameter set.
    pub fn new(k1: f64, k2: f64, k3: f64, p1: f64, p2: f64) -> Self {
        Self { k1, k2, k3, p1, p2 }
    }

    /// Corrects lens distortion for a single 2D pixel coordinate under given intrinsics.
    pub fn undistort_point(&self, px: f64, py: f64, intrinsics: &CameraIntrinsics) -> (f64, f64) {
        let (u, v) = intrinsics.pixel_to_normalized(px, py);
        let r2 = u * u + v * v;
        let r4 = r2 * r2;
        let r6 = r4 * r2;

        let radial = 1.0 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6;
        let du_tangential = 2.0 * self.p1 * u * v + self.p2 * (r2 + 2.0 * u * u);
        let dv_tangential = self.p1 * (r2 + 2.0 * v * v) + 2.0 * self.p2 * u * v;

        let u_corrected = u * radial + du_tangential;
        let v_corrected = v * radial + dv_tangential;

        intrinsics.normalized_to_pixel(u_corrected, v_corrected)
    }

    /// Applies lens undistortion to an entire grayscale or RGB frame.
    pub fn undistort_frame(
        &self,
        frame: &Frame,
        intrinsics: &CameraIntrinsics,
    ) -> Result<Frame, VoxoraError> {
        let width = frame.width as usize;
        let height = frame.height as usize;
        let channels = match frame.format {
            PixelFormat::Grayscale => 1,
            PixelFormat::Rgb8 => 3,
            PixelFormat::Rgba8 => 4,
            PixelFormat::Float32Grayscale => 4,
        };

        let mut out_data = vec![0u8; frame.data.len()];

        for y in 0..height {
            for x in 0..width {
                let (ux, uy) = self.undistort_point(x as f64, y as f64, intrinsics);
                let src_x = ux.round().clamp(0.0, (width - 1) as f64) as usize;
                let src_y = uy.round().clamp(0.0, (height - 1) as f64) as usize;

                for c in 0..channels {
                    out_data[(y * width + x) * channels + c] =
                        frame.data[(src_y * width + src_x) * channels + c];
                }
            }
        }

        Frame::new(frame.width, frame.height, frame.format, out_data)
    }
}

/// 3D Camera Pose representing rotation matrix $R \in SO(3)$ and translation vector $\mathbf{t} \in \mathbb{R}^3$.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraPose {
    /// $3 \times 3$ Rotation matrix
    pub rotation: Matrix3x3,
    /// $3 \times 1$ Translation vector
    pub translation: Vector3,
}

impl Default for CameraPose {
    fn default() -> Self {
        Self { rotation: Matrix3x3::IDENTITY, translation: Vector3::ZERO }
    }
}

impl CameraPose {
    /// Creates a camera pose.
    pub fn new(rotation: Matrix3x3, translation: Vector3) -> Self {
        Self { rotation, translation }
    }

    /// Transforms a 3D world coordinate into local camera coordinates $P_{cam} = R P_{world} + \mathbf{t}$.
    pub fn world_to_camera(&self, p_world: Vector3) -> Vector3 {
        let r_p = self.rotation.mul_vec(p_world);
        Vector3::new(
            r_p.x + self.translation.x,
            r_p.y + self.translation.y,
            r_p.z + self.translation.z,
        )
    }

    /// Transforms a local camera coordinate into 3D world coordinates $P_{world} = R^T (P_{cam} - \mathbf{t})$.
    pub fn camera_to_world(&self, p_cam: Vector3) -> Vector3 {
        let diff = Vector3::new(
            p_cam.x - self.translation.x,
            p_cam.y - self.translation.y,
            p_cam.z - self.translation.z,
        );
        let r_t = Matrix3x3::from_row_major([
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
        r_t.mul_vec(diff)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_intrinsics_projection() {
        let intr = CameraIntrinsics::from_fov(60.0, 640, 480);
        let (u, v) = intr.pixel_to_normalized(320.0, 240.0);
        assert!((u - 0.0).abs() < 1e-5);
        assert!((v - 0.0).abs() < 1e-5);

        let (px, py) = intr.normalized_to_pixel(u, v);
        assert!((px - 320.0).abs() < 1e-5);
        assert!((py - 240.0).abs() < 1e-5);
    }

    #[test]
    fn test_camera_pose_transform() {
        let pose = CameraPose::default();
        let p_world = Vector3::new(1.0, 2.0, 3.0);
        let p_cam = pose.world_to_camera(p_world);
        assert_eq!(p_cam, p_world);
        assert_eq!(pose.camera_to_world(p_cam), p_world);
    }
}
