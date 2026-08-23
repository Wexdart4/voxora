//! Point cloud structures, dense stereo disparity estimation, and DLT 3D triangulation algorithms for Voxora.

#![warn(missing_docs)]

pub mod cloud;
pub mod metrics;
pub mod stereo;
pub mod triangulation;

pub use cloud::{Point3D, PointCloud};
pub use metrics::{DiagnosticsReport, ReconstructionQuality};
pub use stereo::{BlockMatcher, DisparityMap, PlaneSweepStereo};
pub use triangulation::{triangulate_matches, triangulate_point, TriangulationResult};
