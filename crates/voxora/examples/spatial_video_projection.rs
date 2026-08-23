//! Example demonstrating SE(3) pose SLERP interpolation, perspective spatial video projection, bilinear warping, and hole filling.

use voxora::{
    fill_projection_holes, warp_perspective, BlockMatcher, CameraIntrinsics, Matrix3x3, Quaternion,
    SpatialProjector, SyntheticVideoDecoder, TransformSE3, Vector3, VideoDecoder,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Spatial Video Projection & SE(3) Transformations Demo ---");

    // 1. Setup Camera Models & SE(3) Transformations
    let width = 128;
    let height = 128;
    let intrinsics = CameraIntrinsics::from_fov(60.0, width, height);

    let pose_start = TransformSE3::new(Matrix3x3::IDENTITY, Vector3::new(0.0, 0.0, 0.0));
    let pose_end = TransformSE3::new(
        Quaternion::new(0.996, 0.0, 0.087, 0.0).to_rotation_matrix(), // ~10 deg Y rotation
        Vector3::new(0.5, 0.1, 0.2),                                  // SE(3) translation
    );

    println!(
        "SE(3) Initialized: Start Pos=(0,0,0), End Pos=({:.2},{:.2},{:.2})",
        pose_end.translation.x, pose_end.translation.y, pose_end.translation.z
    );

    // 2. Perform SLERP Interpolation of Camera Poses
    let q_start = Quaternion::from_rotation_matrix(&pose_start.rotation);
    let q_end = Quaternion::from_rotation_matrix(&pose_end.rotation);

    for step in 0..=4 {
        let t = step as f64 / 4.0;
        let q_interp = Quaternion::slerp(&q_start, &q_end, t);
        let trans_interp = Vector3::new(
            pose_start.translation.x + t * (pose_end.translation.x - pose_start.translation.x),
            pose_start.translation.y + t * (pose_end.translation.y - pose_start.translation.y),
            pose_start.translation.z + t * (pose_end.translation.z - pose_start.translation.z),
        );
        let pose_interp = TransformSE3::from_quaternion_and_translation(q_interp, trans_interp);

        println!(
            "SLERP Step t={:.2}: Interpolated SE(3) Pos=({:.2},{:.2},{:.2}), Quat=(w={:.3},y={:.3})",
            t, pose_interp.translation.x, pose_interp.translation.y, pose_interp.translation.z,
            q_interp.w, q_interp.y
        );
    }

    // 3. Project 2D Frame + Dense Depth into 3D Spatial Point Cloud
    let mut decoder = SyntheticVideoDecoder::new(width, height, 5, 30.0);
    let frame1 = decoder.next_frame()?.ok_or("Failed to fetch frame 1")?;
    let frame2 = decoder.next_frame()?.ok_or("Failed to fetch frame 2")?;

    let matcher = BlockMatcher::new(7, 16);
    let disp = matcher.compute_disparity(&frame1, &frame2)?;
    let depth_map = matcher.disparity_to_depth(&disp, intrinsics.fx as f32, 0.5);

    let projector = SpatialProjector::new(intrinsics, pose_end);
    let cloud = projector.project_frame(&frame1, &depth_map, 4)?;

    println!(
        "Perspective Spatial Projector: Generated 3D Point Cloud with {} spatial points (stride 4)",
        cloud.len()
    );

    // 4. Perspective Bilinear Image Warping & Disocclusion Hole Filling
    let homography =
        Matrix3x3::from_row_major([1.02, 0.01, 2.0, -0.01, 1.01, 1.0, 0.0001, 0.0001, 1.0]);

    let warped_frame = warp_perspective(&frame1, &homography)?;
    let filled_frame = fill_projection_holes(&warped_frame)?;

    println!(
        "Bilinear Image Warping & Hole Filling: Source {}x{} -> Warped {}x{} -> Disocclusion Filtered {}x{}",
        frame1.width, frame1.height, warped_frame.width, warped_frame.height, filled_frame.width, filled_frame.height
    );

    println!("--- Spatial Video Projection Completed ---");
    Ok(())
}
