//! Example demonstrating DLT triangulation, dense stereo disparity estimation, and point cloud spatial filtering.

use voxora::{
    triangulate_matches, BlockMatcher, BruteForceMatcher, CameraIntrinsics, CameraPose,
    FeatureDetector, HarrisCornerDetector, Matrix3x3, Point3D, PointCloud, SyntheticVideoDecoder,
    Vector3, VideoDecoder,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Spatial 3D Reconstruction & Point Cloud Pipeline ---");

    // 1. Setup Camera Models & Stereo Baseline
    let width = 128;
    let height = 128;
    let intrinsics = CameraIntrinsics::from_fov(60.0, width, height);

    let pose1 = CameraPose::default();
    let pose2 = CameraPose::new(
        Matrix3x3::IDENTITY,
        Vector3::new(0.5, 0.0, 0.0), // 0.5 unit stereo baseline
    );

    println!("Stereo Rig Initialized: 60deg FOV, Baseline=0.5u, Resolution={}x{}", width, height);

    // 2. Fetch Frames & Perform Feature Detection/Matching
    let mut decoder = SyntheticVideoDecoder::new(width, height, 5, 30.0);
    let frame1 = decoder.next_frame()?.ok_or("Failed to fetch frame 1")?;
    let frame2 = decoder.next_frame()?.ok_or("Failed to fetch frame 2")?;

    let detector = HarrisCornerDetector { threshold: 2000.0, k: 0.04, max_features: 80 };
    let query_pts = detector.detect(&frame1);
    let train_pts = detector.detect(&frame2);

    let matcher = BruteForceMatcher::new(0.85, Some(50.0), true);
    let matches = matcher.match_features(&query_pts, &train_pts);

    println!(
        "Corner Detection & Matching: Query={:?}, Train={:?}, Cross-Checked Matches={}",
        query_pts.len(),
        train_pts.len(),
        matches.len()
    );

    // 3. Multi-View DLT 3D Point Triangulation
    let tri_results = triangulate_matches(
        &matches,
        &query_pts,
        &train_pts,
        &pose1,
        &pose2,
        &intrinsics,
        15.0, // Max reprojection error threshold (pixels)
    );

    println!(
        "DLT Triangulation: Successfully reconstructed {} valid 3D points from matches",
        tri_results.len()
    );

    // Build raw point cloud container
    let mut cloud = PointCloud::new();
    for res in &tri_results {
        cloud.push(Point3D::new(res.point, [200, 200, 200], 0.95));
    }

    // Add extra synthetic points for filter verification
    cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 5.0), [255, 0, 0], 1.0));
    cloud.push(Point3D::new(Vector3::new(0.01, 0.01, 5.01), [255, 0, 0], 1.0)); // Duplicate
    cloud.push(Point3D::new(Vector3::new(100.0, 100.0, 100.0), [0, 0, 255], 0.1)); // Outlier

    println!("Raw Point Cloud Size: {} points", cloud.len());

    // 4. Point Cloud Spatial Downsampling & Outlier Filtering
    let merged_cloud = cloud.merge_duplicate_points(0.05);
    println!("After Duplicate Merging (dist <= 0.05u): {} points", merged_cloud.len());

    let voxel_cloud = merged_cloud.voxel_grid_filter(0.2);
    println!("After 3D Voxel Grid Filter (cell = 0.2u): {} points", voxel_cloud.len());

    let clean_cloud = voxel_cloud.statistical_outlier_removal(2, 1.5);
    println!("After Statistical Outlier Removal (k=2, 1.5std): {} points", clean_cloud.len());

    // 5. Dense Stereo SAD Disparity & Depth Map Estimation
    let block_matcher = BlockMatcher::new(7, 16);
    let disp_map = block_matcher.compute_disparity(&frame1, &frame2)?;
    let depth_map = block_matcher.disparity_to_depth(&disp_map, intrinsics.fx as f32, 0.5);

    let valid_depth_pixels = depth_map.iter().filter(|&&d| d > 0.0).count();
    println!(
        "Dense Stereo SAD Matching: Computed {}x{} Disparity Map ({} valid depth pixels)",
        disp_map.width, disp_map.height, valid_depth_pixels
    );

    println!("--- Spatial 3D Reconstruction Pipeline Execution Complete ---");
    Ok(())
}
