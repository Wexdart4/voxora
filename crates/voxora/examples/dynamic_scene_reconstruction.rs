//! Example demonstrating scene cut detection, dynamic object filtering, temporal geometry stabilization, and quality metrics reporting.

use voxora::{
    CutType, DiagnosticsReport, DynamicMaskEstimator, Matrix3x3, Point3D, PointCloud,
    SceneCutDetector, SyntheticVideoDecoder, TemporalStabilizer, Vector3, VideoDecoder,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Dynamic Scene Handling & Quality Diagnostics Demo ---");

    // 1. Initialize Scene Cut Detector & Temporal Geometry Stabilizer
    let mut cut_detector = SceneCutDetector::new(0.3, 0.2);
    let stabilizer = TemporalStabilizer::new(0.6);
    let mask_estimator = DynamicMaskEstimator::new(2.0);

    let start_pts = [(10.0, 10.0), (50.0, 50.0)];
    let end_pts = [(10.5, 10.2), (70.0, 80.0)];
    let static_mask =
        mask_estimator.filter_static_features(&start_pts, &end_pts, &Matrix3x3::IDENTITY);

    let mut decoder = SyntheticVideoDecoder::new(128, 128, 10, 30.0);

    println!(
        "Initialized Scene Cut Detector, Temporal Stabilizer (alpha=0.6), and Dynamic Mask Estimator (isolated {} static points).",
        static_mask.iter().filter(|&&s| s).count()
    );

    // 2. Process Video Sequence & Monitor Scene Transitions
    let mut frame_count = 0;
    while let Some(frame) = decoder.next_frame()? {
        frame_count += 1;
        let match_ratio = if frame_count == 6 { 0.05 } else { 0.85 }; // Simulate scene cut at frame 6

        let transition = cut_detector.detect_cut(&frame, match_ratio)?;

        match transition {
            CutType::None => {
                println!(
                    "Frame #{}: Continuous tracking (match_ratio={:.2})",
                    frame_count, match_ratio
                );
            }
            CutType::HardCut => {
                println!(
                    "Frame #{}: [ALERT] Hard Cut detected! Resetting feature tracking and trajectory state.",
                    frame_count
                );
                cut_detector.reset();
            }
            CutType::Fade => {
                println!(
                    "Frame #{}: [ALERT] Transition / Fade detected! Resetting tracking state.",
                    frame_count
                );
                cut_detector.reset();
            }
        }
    }

    // 3. Temporal Position Stabilization Test
    let raw_positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.2, 0.05, 1.01),
        Vector3::new(0.1, -0.02, 0.99),
        Vector3::new(0.4, 0.08, 1.02),
    ];
    let smoothed_positions = stabilizer.smooth_sequence(&raw_positions);

    println!(
        "Temporal Geometry Stabilizer: Smoothed {} sequential spatial positions (alpha=0.6)",
        smoothed_positions.len()
    );
    for (i, (raw, sm)) in raw_positions.iter().zip(smoothed_positions.iter()).enumerate() {
        println!(
            "  Step #{}: Raw=({:.2},{:.2},{:.2}) -> Smoothed=({:.2},{:.2},{:.2})",
            i + 1,
            raw.x,
            raw.y,
            raw.z,
            sm.x,
            sm.y,
            sm.z
        );
    }

    // 4. Evaluate Spatial Reconstruction Quality Metrics & Report
    let mut cloud = PointCloud::new();
    cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 1.0), [255, 0, 0], 0.95));
    cloud.push(Point3D::new(Vector3::new(1.0, 0.0, 1.2), [0, 255, 0], 0.88));
    cloud.push(Point3D::new(Vector3::new(-1.0, 0.5, 0.9), [0, 0, 255], 0.92));

    let reproj_errors = vec![0.42, 0.68, 0.35];
    let report = DiagnosticsReport::generate(&cloud, &reproj_errors);

    println!("--- Spatial Reconstruction Diagnostics Report ---");
    println!("Total Spatial Points: {}", report.total_points);
    println!("Average Point Confidence: {:.2}", report.average_confidence);
    println!("Mean Reprojection Error: {:.3} px", report.quality.mean_reprojection_error);
    println!("Geometric Consistency: {:.2}", report.quality.geometric_consistency);
    println!("Temporal Stability: {:.2}", report.quality.temporal_stability);
    println!("Overall Quality Confidence Score: {:.2}", report.quality.overall_confidence);
    println!("--- Dynamic Scene Reconstruction Demo Completed ---");

    Ok(())
}
