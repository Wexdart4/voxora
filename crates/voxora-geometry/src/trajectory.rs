//! Camera trajectory tracking, pose accumulation, motion velocity estimation, and trajectory smoothing.

use crate::camera::CameraPose;
use voxora_math::Vector3;

/// Motion category classification for a camera frame sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionCategory {
    /// Camera is stationary (translation & rotation near zero)
    StaticCamera,
    /// Smooth continuous camera movement
    SmoothMotion,
    /// Sudden rapid angular rotation detected
    SuddenRotation,
    /// Motion anomaly or tracking loss
    TrackingLoss,
}

/// Accumulates camera pose trajectory across frame timeline.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraTrajectory {
    /// Ordered vector of global camera poses for each frame
    pub poses: Vec<CameraPose>,
}

impl Default for CameraTrajectory {
    fn default() -> Self {
        Self { poses: vec![CameraPose::default()] }
    }
}

impl CameraTrajectory {
    /// Creates a new camera trajectory initialized with default origin pose.
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends relative motion transformation $\Delta P$ to global trajectory timeline.
    pub fn add_relative_pose(&mut self, relative_pose: CameraPose) {
        let last_pose = self.poses.last().cloned().unwrap_or_default();

        let new_translation = Vector3::new(
            last_pose.translation.x + relative_pose.translation.x,
            last_pose.translation.y + relative_pose.translation.y,
            last_pose.translation.z + relative_pose.translation.z,
        );

        let new_pose = CameraPose::new(last_pose.rotation, new_translation);
        self.poses.push(new_pose);
    }

    /// Calculates total Euclidean distance traveled by camera origin.
    pub fn total_distance(&self) -> f64 {
        let mut dist = 0.0;
        for window in self.poses.windows(2) {
            let dx = window[1].translation.x - window[0].translation.x;
            let dy = window[1].translation.y - window[0].translation.y;
            let dz = window[1].translation.z - window[0].translation.z;
            dist += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        dist
    }

    /// Classifies overall motion characteristics of the trajectory segment.
    pub fn classify_motion(&self) -> MotionCategory {
        if self.poses.len() < 2 {
            return MotionCategory::StaticCamera;
        }

        let dist = self.total_distance();
        let avg_speed = dist / (self.poses.len() - 1) as f64;

        if avg_speed < 1e-4 {
            MotionCategory::StaticCamera
        } else if avg_speed > 50.0 {
            MotionCategory::TrackingLoss
        } else {
            MotionCategory::SmoothMotion
        }
    }
}

/// Moving-window trajectory filter for trajectory smoothing.
#[derive(Debug, Clone)]
pub struct TrajectoryFilter {
    /// Smoothing window radius (in frames)
    pub window_size: usize,
}

impl Default for TrajectoryFilter {
    fn default() -> Self {
        Self { window_size: 5 }
    }
}

impl TrajectoryFilter {
    /// Creates a trajectory filter with specified window size.
    pub fn new(window_size: usize) -> Self {
        Self { window_size }
    }

    /// Smooths input camera trajectory using moving window averaging while preserving raw trajectory.
    pub fn smooth_trajectory(&self, trajectory: &CameraTrajectory) -> CameraTrajectory {
        let n = trajectory.poses.len();
        if n <= 2 {
            return trajectory.clone();
        }

        let mut smoothed_poses = Vec::with_capacity(n);
        let half_w = (self.window_size / 2) as i32;

        for i in 0..n {
            let mut sum_tx = 0.0;
            let mut sum_ty = 0.0;
            let mut sum_tz = 0.0;
            let mut count = 0;

            for w in -half_w..=half_w {
                let idx = (i as i32 + w).clamp(0, (n - 1) as i32) as usize;
                sum_tx += trajectory.poses[idx].translation.x;
                sum_ty += trajectory.poses[idx].translation.y;
                sum_tz += trajectory.poses[idx].translation.z;
                count += 1;
            }

            let avg_trans =
                Vector3::new(sum_tx / count as f64, sum_ty / count as f64, sum_tz / count as f64);

            smoothed_poses.push(CameraPose::new(trajectory.poses[i].rotation, avg_trans));
        }

        CameraTrajectory { poses: smoothed_poses }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trajectory_accumulation_and_smoothing() {
        let mut traj = CameraTrajectory::new();
        traj.add_relative_pose(CameraPose::new(
            voxora_math::Matrix3x3::IDENTITY,
            Vector3::new(1.0, 0.0, 0.0),
        ));
        traj.add_relative_pose(CameraPose::new(
            voxora_math::Matrix3x3::IDENTITY,
            Vector3::new(1.0, 0.0, 0.0),
        ));

        assert_eq!(traj.poses.len(), 3);
        assert!((traj.total_distance() - 2.0).abs() < 1e-5);
        assert_eq!(traj.classify_motion(), MotionCategory::SmoothMotion);

        let filter = TrajectoryFilter::new(3);
        let smoothed = filter.smooth_trajectory(&traj);
        assert_eq!(smoothed.poses.len(), 3);
    }
}
