//! Example demonstrating image preprocessing, edge extraction, corner detection, and descriptors.

use voxora::{
    canny_edge_detector, gaussian_blur, FastFeatureDetector, FeatureDetector, HarrisCornerDetector,
    ShiTomasiDetector, SpatialGridFilter, SyntheticVideoDecoder, VideoDecoder,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Feature Detection & Edge Extraction Demo ---");

    // 1. Generate synthetic frame from video decoder
    let mut decoder = SyntheticVideoDecoder::new(128, 128, 1, 30.0);
    let frame = decoder.next_frame()?.ok_or("Failed to fetch synthetic frame")?;
    println!("Input Frame: {}x{} RGB", frame.width, frame.height);

    // 2. Preprocessing & Edge Detection
    let blurred = gaussian_blur(&frame, 5, 1.2)?;
    let edges = canny_edge_detector(&frame, 30.0, 80.0)?;
    let edge_pixels = edges.data.iter().filter(|&&p| p > 0).count();
    println!("Processed Gaussian Blurred Frame (5x5, sigma=1.2) & Canny Edges (30/80 threshold, {} edge pixels)", edge_pixels);

    // 3. Harris Corner Detection
    let harris = HarrisCornerDetector { threshold: 2000.0, k: 0.04, max_features: 100 };
    let harris_pts = harris.detect(&blurred);
    println!("Harris Corners Detected: {}", harris_pts.len());

    // 4. Shi-Tomasi Corner Detection
    let shi_tomasi = ShiTomasiDetector { threshold: 200.0, max_features: 100 };
    let st_pts = shi_tomasi.detect(&blurred);
    println!("Shi-Tomasi Corners Detected: {}", st_pts.len());

    // 5. FAST Corner Detection
    let fast = FastFeatureDetector { threshold: 15, contiguous_pixels: 9 };
    let fast_pts = fast.detect(&blurred);
    println!("FAST Corners Detected: {}", fast_pts.len());

    // 6. Spatial Grid Filtering (Grid NMS)
    let grid_filter = SpatialGridFilter::new(4, 4, 3);
    let filtered_harris = grid_filter.filter(&harris_pts, frame.width, frame.height);
    println!(
        "Grid Filtered Harris Corners (4x4 grid, max 3/cell): {} features remaining",
        filtered_harris.len()
    );

    if let Some(first_pt) = filtered_harris.first() {
        println!(
            "Sample Feature Point: pos=({:.1}, {:.1}), response={:.2}, binary_descriptor_attached={}",
            first_pt.x,
            first_pt.y,
            first_pt.response,
            first_pt.descriptor.is_some()
        );
    }

    println!("--- Feature Detection Demo Completed ---");
    Ok(())
}
