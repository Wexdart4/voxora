//! Basic Voxora API example pipeline usage.

use voxora::core::{Frame, PixelFormat};
use voxora::geometry::CameraIntrinsics;
use voxora::math::Vector3;
use voxora::reconstruction::{Point3D, PointCloud};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Spatial Pipeline Initializing ---");

    // 1. Initialize camera intrinsics (1920x1080 resolution, f ~ 1000px)
    let intrinsics = CameraIntrinsics::new(1000.0, 1000.0, 960.0, 540.0);
    let k_matrix = intrinsics.to_matrix();
    println!("Camera Matrix K:\n{:?}", k_matrix);

    // 2. Create mock video frame (100x100 RGB)
    let frame_data = vec![128u8; 100 * 100 * 3];
    let frame = Frame::new(100, 100, PixelFormat::Rgb8, frame_data)?;
    println!("Loaded Frame: {}x{} ({:?})", frame.width, frame.height, frame.format);

    // 3. Initialize Point Cloud and insert sample deterministic 3D point
    let mut cloud = PointCloud::new();
    let point = Point3D::new(Vector3::new(0.0, 0.0, 5.0), [255, 0, 0], 1.0);
    cloud.push(point);

    println!("Reconstructed Point Cloud Size: {} points", cloud.len());
    println!("Sample 3D Point: {:?}", cloud.points[0]);

    println!("--- Pipeline Execution Complete ---");
    Ok(())
}
