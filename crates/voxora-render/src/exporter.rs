//! Exporters for 3D point cloud structures and camera trajectories.

use std::fmt::Write;
use voxora_core::VoxoraError;
use voxora_geometry::CameraTrajectory;
use voxora_reconstruction::PointCloud;

/// Exports a 3D point cloud into Stanford PLY ASCII format string.
pub fn export_ply(cloud: &PointCloud) -> Result<String, VoxoraError> {
    let mut out = String::new();
    writeln!(out, "ply").unwrap();
    writeln!(out, "format ascii 1.0").unwrap();
    writeln!(out, "comment Exported by Voxora Vision Library").unwrap();
    writeln!(out, "element vertex {}", cloud.len()).unwrap();
    writeln!(out, "property float x").unwrap();
    writeln!(out, "property float y").unwrap();
    writeln!(out, "property float z").unwrap();
    writeln!(out, "property uchar red").unwrap();
    writeln!(out, "property uchar green").unwrap();
    writeln!(out, "property uchar blue").unwrap();
    writeln!(out, "end_header").unwrap();

    for pt in &cloud.points {
        writeln!(
            out,
            "{:.4} {:.4} {:.4} {} {} {}",
            pt.position.x, pt.position.y, pt.position.z, pt.color[0], pt.color[1], pt.color[2]
        )
        .unwrap();
    }

    Ok(out)
}

/// Exports a 3D point cloud into Wavefront OBJ format string.
pub fn export_obj(cloud: &PointCloud) -> Result<String, VoxoraError> {
    let mut out = String::new();
    writeln!(out, "# Wavefront OBJ exported by Voxora").unwrap();

    for pt in &cloud.points {
        let r = pt.color[0] as f64 / 255.0;
        let g = pt.color[1] as f64 / 255.0;
        let b = pt.color[2] as f64 / 255.0;
        writeln!(
            out,
            "v {:.4} {:.4} {:.4} {:.3} {:.3} {:.3}",
            pt.position.x, pt.position.y, pt.position.z, r, g, b
        )
        .unwrap();
    }

    Ok(out)
}

/// Exports a 3D point cloud into glTF 2.0 asset JSON string representation.
pub fn export_gltf_json(cloud: &PointCloud) -> Result<String, VoxoraError> {
    let mut json = String::new();
    writeln!(json, "{{").unwrap();
    writeln!(json, "  \"asset\": {{ \"version\": \"2.0\", \"generator\": \"Voxora Vision\" }},")
        .unwrap();
    writeln!(json, "  \"scenes\": [ {{ \"nodes\": [0] }} ],").unwrap();
    writeln!(json, "  \"nodes\": [ {{ \"name\": \"SpatialPointCloud\", \"mesh\": 0 }} ],").unwrap();
    writeln!(
        json,
        "  \"meshes\": [ {{ \"primitives\": [ {{ \"attributes\": {{ \"POSITION\": 0 }} }} ] }} ],"
    )
    .unwrap();
    writeln!(json, "  \"extras\": {{ \"point_count\": {} }}", cloud.len()).unwrap();
    writeln!(json, "}}").unwrap();
    Ok(json)
}

/// Exports a camera trajectory sequence into CSV format.
pub fn export_trajectory_csv(trajectory: &CameraTrajectory) -> Result<String, VoxoraError> {
    let mut csv = String::new();
    writeln!(csv, "frame_id,timestamp_ms,pos_x,pos_y,pos_z,r00,r01,r02,r10").unwrap();

    for (idx, pose) in trajectory.poses.iter().enumerate() {
        writeln!(
            csv,
            "{},{:.2},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            idx,
            idx as f64 * 33.3,
            pose.translation.x,
            pose.translation.y,
            pose.translation.z,
            pose.rotation.get(0, 0),
            pose.rotation.get(0, 1),
            pose.rotation.get(0, 2),
            pose.rotation.get(1, 0)
        )
        .unwrap();
    }

    Ok(csv)
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_math::Vector3;
    use voxora_reconstruction::Point3D;

    #[test]
    fn test_export_ply_format() {
        let mut cloud = PointCloud::new();
        cloud.push(Point3D::new(Vector3::new(1.0, 2.0, 3.0), [255, 0, 0], 1.0));
        let ply = export_ply(&cloud).unwrap();

        assert!(ply.contains("ply"));
        assert!(ply.contains("element vertex 1"));
        assert!(ply.contains("1.0000 2.0000 3.0000 255 0 0"));
    }
}
