//! Example demonstrating camera intrinsics, Essential matrix pose recovery, and camera trajectory tracking.

use voxora::{
    recover_pose, BruteForceMatcher, CameraIntrinsics, CameraPose, CameraTrajectory,
    EssentialMatrix, FeatureDetector, HarrisCornerDetector, LensDistortion, Matrix3x3,
    SyntheticVideoDecoder, TrajectoryFilter, Vector3, VideoDecoder,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Camera Geometry & Motion Estimation Demo ---");

    // 1. Setup Camera Model & Lens Distortion Parameters
    let width = 128;
    let height = 128;
    let intrinsics = CameraIntrinsics::from_fov(60.0, width, height);
    let distortion = LensDistortion::new(-0.1, 0.01, 0.0, 0.001, 0.0);

    println!(
        "Camera Intrinsics (60deg FOV, {}x{}): fx={:.1}, fy={:.1}, cx={:.1}, cy={:.1}",
        width, height, intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy
    );

    let sample_pt = (320.0, 240.0);
    let corrected_pt = distortion.undistort_point(sample_pt.0, sample_pt.1, &intrinsics);
    println!(
        "Lens Undistortion: Center Pixel ({:.1}, {:.1}) -> Corrected ({:.1}, {:.1})",
        sample_pt.0, sample_pt.1, corrected_pt.0, corrected_pt.1
    );

    // 2. Load Synthetic Video Frames
    let mut decoder = SyntheticVideoDecoder::new(width, height, 5, 30.0);
    let frame1 = decoder.next_frame()?.ok_or("Failed to fetch frame 1")?;
    let frame2 = decoder.next_frame()?.ok_or("Failed to fetch frame 2")?;

    // 3. Feature Extraction & Matching
    let detector = HarrisCornerDetector { threshold: 2000.0, k: 0.04, max_features: 100 };
    let query_pts = detector.detect(&frame1);
    let train_pts = detector.detect(&frame2);

    let matcher = BruteForceMatcher::new(0.85, Some(50.0), true);
    let matches = matcher.match_features(&query_pts, &train_pts);
    println!(
        "Detected {} & {} features -> {} Cross-Checked Matches",
        query_pts.len(),
        train_pts.len(),
        matches.len()
    );

    // 4. Recover Camera Pose from Essential Matrix via Cheirality Check
    let essential = EssentialMatrix::new(Matrix3x3::IDENTITY);
    if let Some((recovered_pose, cheirality_inliers)) =
        recover_pose(&essential, &matches, &query_pts, &train_pts, &intrinsics)
    {
        println!(
            "Recovered Camera Relative Pose: Translation=({:.2}, {:.2}, {:.2}), Valid Depth Inliers={}/{}",
            recovered_pose.translation.x,
            recovered_pose.translation.y,
            recovered_pose.translation.z,
            cheirality_inliers,
            matches.len()
        );
    }

    // 5. Track Camera Trajectory Across Frames
    let mut trajectory = CameraTrajectory::new();
    let frame_translations = [
        Vector3::new(0.5, 0.0, 0.1),
        Vector3::new(0.6, 0.1, 0.0),
        Vector3::new(0.4, -0.1, 0.2),
        Vector3::new(0.5, 0.0, 0.1),
    ];

    for (idx, trans) in frame_translations.iter().enumerate() {
        let rel_pose = CameraPose::new(Matrix3x3::IDENTITY, *trans);
        trajectory.add_relative_pose(rel_pose);
        println!(
            "Frame {}: Accumulated Camera Position=({:.2}, {:.2}, {:.2})",
            idx + 1,
            trajectory.poses.last().unwrap().translation.x,
            trajectory.poses.last().unwrap().translation.y,
            trajectory.poses.last().unwrap().translation.z
        );
    }

    let total_dist = trajectory.total_distance();
    let motion_cat = trajectory.classify_motion();
    println!(
        "Camera Motion Trajectory Summary: Poses={}, Total Distance={:.2} units, Classification={:?}",
        trajectory.poses.len(),
        total_dist,
        motion_cat
    );

    // 6. Smooth Camera Trajectory
    let filter = TrajectoryFilter::new(3);
    let smoothed_traj = filter.smooth_trajectory(&trajectory);
    println!(
        "Trajectory Filter Applied (Moving Window Size 3): Smoothed Poses={}",
        smoothed_traj.poses.len()
    );

    println!("--- Camera Motion Estimation Completed ---");
    Ok(())
}
