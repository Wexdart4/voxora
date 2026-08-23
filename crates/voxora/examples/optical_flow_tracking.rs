//! Example demonstrating Pyramidal Lucas-Kanade optical flow tracking and RANSAC homography estimation.

use voxora::{
    forward_backward_flow_check, pyramidal_lucas_kanade, ransac_homography, BruteForceMatcher,
    FeatureDetector, FeatureTrack, HarrisCornerDetector, RansacOptions, SyntheticVideoDecoder,
    VideoDecoder,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Optical Flow & RANSAC Tracking Demo ---");

    // 1. Generate 2 consecutive synthetic video frames
    let mut decoder = SyntheticVideoDecoder::new(128, 128, 5, 30.0);
    let frame1 = decoder.next_frame()?.ok_or("Failed to fetch frame 1")?;
    let frame2 = decoder.next_frame()?.ok_or("Failed to fetch frame 2")?;

    println!("Input Video Stream: 128x128 @ 30 FPS");

    // 2. Detect corners on frame 1
    let detector = HarrisCornerDetector { threshold: 2000.0, k: 0.04, max_features: 80 };
    let query_pts = detector.detect(&frame1);
    let train_pts = detector.detect(&frame2);

    println!(
        "Detected {} features in Frame 1, {} features in Frame 2",
        query_pts.len(),
        train_pts.len()
    );

    // 3. Match features between Frame 1 & Frame 2
    let matcher = BruteForceMatcher::new(0.85, Some(40.0), true);
    let initial_matches = matcher.match_features(&query_pts, &train_pts);
    println!("Brute-Force Matches (Cross-Checked & Ratio Tested): {}", initial_matches.len());

    // 4. Robust RANSAC Homography Estimation
    let ransac_opts =
        RansacOptions { max_iterations: 200, inlier_threshold: 3.0, min_inlier_ratio: 0.3 };

    if let Some(ransac_res) =
        ransac_homography(&initial_matches, &query_pts, &train_pts, &ransac_opts)
    {
        println!(
            "RANSAC Homography Fit: Status={:?}, Inliers={}/{} ({:.1}%)",
            ransac_res.status,
            ransac_res.inlier_indices.len(),
            initial_matches.len(),
            ransac_res.inlier_ratio * 100.0
        );
    }

    // 5. Pyramidal Lucas-Kanade Optical Flow Tracking & Forward-Backward Check
    let flows = pyramidal_lucas_kanade(&frame1, &frame2, &query_pts, 3, 7, 10)?;
    let tracked_count = flows.iter().filter(|f| f.is_tracked).count();
    println!("Lucas-Kanade Pyramidal Flow: Tracked {}/{} points", tracked_count, query_pts.len());

    let valid_mask = forward_backward_flow_check(&frame1, &frame2, &query_pts, 1.5)?;
    let valid_count = valid_mask.iter().filter(|&&v| v).count();
    println!("Forward-Backward Consistency Check: {} valid tracks verified", valid_count);

    // 6. Initialize Feature Tracks
    let mut tracks = Vec::new();
    for (i, pt) in query_pts.iter().enumerate() {
        if valid_mask[i] {
            let mut track = FeatureTrack::new(i, 0, pt.clone());
            let flow = flows[i];
            let mut tracked_pt = pt.clone();
            tracked_pt.x += flow.dx;
            tracked_pt.y += flow.dy;
            track.add_point(1, tracked_pt);
            tracks.push(track);
        }
    }

    println!("Active Multi-Frame Feature Tracks Initialized: {}", tracks.len());
    println!("--- Optical Flow & RANSAC Tracking Completed ---");
    Ok(())
}
