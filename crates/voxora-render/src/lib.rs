//! 3D rendering abstractions, spatial video projection, perspective warping, and hole filling for Voxora.

#![warn(missing_docs)]

pub mod exporter;
pub mod projection;
pub mod software;
pub mod stereo_output;
pub mod warping;

pub use exporter::{export_gltf_json, export_obj, export_ply, export_trajectory_csv};
pub use projection::SpatialProjector;
pub use software::SoftwareRenderer;
pub use stereo_output::{StereoCameraRig, StereoFrameComposer, StereoLayout};
pub use warping::{fill_projection_holes, sample_bilinear, warp_perspective};

use voxora_geometry::CameraPose;

/// Abstract virtual camera for observing reconstructed 3D spatial representations.
#[derive(Debug, Clone, PartialEq)]
pub struct VirtualCamera {
    /// Spatial pose of the virtual camera
    pub pose: CameraPose,
    /// Field of view in degrees
    pub fov_degrees: f32,
}

impl Default for VirtualCamera {
    fn default() -> Self {
        Self { pose: CameraPose::default(), fov_degrees: 60.0 }
    }
}
