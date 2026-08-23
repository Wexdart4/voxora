//! Camera geometry, homography, epipolar geometry, lens distortion, and camera motion tracking.

#![warn(missing_docs)]

pub mod camera;
pub mod epipolar;
pub mod se3;
pub mod trajectory;

pub use camera::{CameraIntrinsics, CameraPose, LensDistortion};
pub use epipolar::{recover_pose, EssentialMatrix};
pub use se3::{Quaternion, TransformSE3};
pub use trajectory::{CameraTrajectory, MotionCategory, TrajectoryFilter};
