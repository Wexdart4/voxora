//! Workspace integration tests for Voxora pipeline components.

use voxora::core::{Frame, PixelFormat, VoxoraError};
use voxora::geometry::{CameraIntrinsics, CameraPose};
use voxora::math::{Vector3, Matrix3x3, EPSILON};
use voxora::reconstruction::{Point3D, PointCloud};

#[test]
fn test_workspace_math_and_geometry_integration() {
    let vec_a = Vector3::new(1.0, 0.0, 0.0);
    let vec_b = Vector3::new(0.0, 1.0, 0.0);
    let vec_c = vec_a.cross(&vec_b);

    assert!((vec_c.z - 1.0).abs() < EPSILON);

    let pose = CameraPose::default();
    let transformed = pose.rotation.mul_vec(vec_c);
    assert_eq!(transformed, vec_c);
}

#[test]
fn test_workspace_frame_and_reconstruction_integration() {
    let intrinsics = CameraIntrinsics::new(800.0, 800.0, 320.0, 240.0);
    let k = intrinsics.to_matrix();
    assert_eq!(k.data[0], 800.0);

    let frame = Frame::new(10, 10, PixelFormat::Grayscale, vec![0u8; 100]);
    assert!(frame.is_ok());

    let mut cloud = PointCloud::new();
    cloud.push(Point3D::new(Vector3::new(1.0, 2.0, 3.0), [100, 150, 200], 0.95));

    assert_eq!(cloud.len(), 1);
}

#[test]
fn test_invalid_frame_buffer_error_handling() {
    let res = Frame::new(10, 10, PixelFormat::Rgb8, vec![0u8; 10]);
    assert!(matches!(res, Err(VoxoraError::InvalidFrameDimensions { expected: 300, actual: 10 })));
}
