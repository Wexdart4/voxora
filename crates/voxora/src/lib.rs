//! Voxora: Deterministic 3D Video Projection in Pure Rust.
//!
//! Voxora is a pure-Rust computational vision library for transforming ordinary video
//! into spatial 3D representations using deterministic mathematics and geometric algorithms
//! without relying on pretrained neural networks or machine-learning inference models.

#![warn(missing_docs)]

pub use voxora_core as core;
pub use voxora_geometry as geometry;
pub use voxora_math as math;
pub use voxora_reconstruction as reconstruction;
pub use voxora_render as render;
pub use voxora_vision as vision;

pub use voxora_core::decoder::{
    ColorSpace, ImageSequenceDecoder, SyntheticVideoDecoder, VideoDecoder, VideoMetadata,
    VideoReader,
};
pub use voxora_core::{BoundedFrameQueue, Frame, FrameF32, PixelFormat, VoxoraError};

pub use voxora_geometry::camera::{CameraIntrinsics, CameraPose, LensDistortion};
pub use voxora_geometry::epipolar::{recover_pose, EssentialMatrix};
pub use voxora_geometry::se3::{Quaternion, TransformSE3};
pub use voxora_geometry::trajectory::{CameraTrajectory, MotionCategory, TrajectoryFilter};
pub use voxora_math::{
    least_squares_solve, sanitize_float, svd_3x3, Matrix3x3, SvdResult3x3, Vector3,
};
pub use voxora_reconstruction::cloud::{Point3D, PointCloud};
pub use voxora_reconstruction::metrics::{DiagnosticsReport, ReconstructionQuality};
pub use voxora_reconstruction::stereo::{BlockMatcher, DisparityMap};
pub use voxora_reconstruction::triangulation::{
    triangulate_matches, triangulate_point, TriangulationResult,
};
pub use voxora_render::exporter::{
    export_gltf_json, export_obj, export_ply, export_trajectory_csv,
};
pub use voxora_render::projection::SpatialProjector;
pub use voxora_render::software::SoftwareRenderer;
pub use voxora_render::stereo_output::{StereoCameraRig, StereoFrameComposer, StereoLayout};
pub use voxora_render::warping::{fill_projection_holes, sample_bilinear, warp_perspective};
pub use voxora_render::VirtualCamera;
pub use voxora_vision::segmentation::{CutType, DynamicMaskEstimator, SceneCutDetector};
pub use voxora_vision::temporal::TemporalStabilizer;

pub use voxora_geometry as geometry_crate;
pub use voxora_vision::descriptor::{
    euclidean_distance, hamming_distance, BinaryDescriptor, Descriptor, PatchDescriptor,
};
pub use voxora_vision::edge::{canny_edge_detector, scharr_operator, sobel_operator, GradientMap};
pub use voxora_vision::filter::{
    box_filter, contrast_brightness_normalize, gaussian_blur, histogram_equalization,
    median_filter, GaussianPyramid,
};
pub use voxora_vision::flow::{
    forward_backward_flow_check, pyramidal_lucas_kanade, FeatureTrack, FlowVector,
};
pub use voxora_vision::matching::{BruteForceMatcher, FeatureMatch};
pub use voxora_vision::ransac::{
    ransac_fundamental_matrix, ransac_homography, RansacOptions, RansacResult, TrackingStatus,
};
pub use voxora_vision::{
    FastFeatureDetector, FeatureDetector, FeaturePoint, HarrisCornerDetector, ShiTomasiDetector,
    SpatialGridFilter,
};

/// Crate metadata and version information.
pub mod info {
    /// Library version string.
    pub const VERSION: &str = env!("CARGO_PKG_VERSION");
    /// Library description.
    pub const DESCRIPTION: &str = "Deterministic 3D Video Projection in Pure Rust";
}
