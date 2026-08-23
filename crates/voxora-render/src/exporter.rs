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

fn base64_encode(data: &[u8]) -> String {
    const CHARSET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    let mut i = 0;
    while i < data.len() {
        let b0 = data[i] as u32;
        let b1 = if i + 1 < data.len() { data[i + 1] as u32 } else { 0 };
        let b2 = if i + 2 < data.len() { data[i + 2] as u32 } else { 0 };

        let triple = (b0 << 16) | (b1 << 8) | b2;

        out.push(CHARSET[((triple >> 18) & 63) as usize] as char);
        out.push(CHARSET[((triple >> 12) & 63) as usize] as char);
        if i + 1 < data.len() {
            out.push(CHARSET[((triple >> 6) & 63) as usize] as char);
        } else {
            out.push('=');
        }
        if i + 2 < data.len() {
            out.push(CHARSET[(triple & 63) as usize] as char);
        } else {
            out.push('=');
        }

        i += 3;
    }
    out
}

/// Exports a 3D point cloud into glTF 2.0 asset JSON string representation.
pub fn export_gltf_json(cloud: &PointCloud) -> Result<String, VoxoraError> {
    let count = cloud.len();
    if count == 0 {
        let json = format!(
            "{{\n  \"asset\": {{ \"version\": \"2.0\", \"generator\": \"Voxora Vision\" }},\n  \"scene\": 0,\n  \"scenes\": [ {{ \"nodes\": [] }} ],\n  \"nodes\": []\n}}"
        );
        return Ok(json);
    }

    let mut min_pos = [f32::MAX, f32::MAX, f32::MAX];
    let mut max_pos = [f32::MIN, f32::MIN, f32::MIN];

    let mut buffer_bytes = Vec::with_capacity(count * 16);

    // Write position floats (VEC3 f32)
    for pt in &cloud.points {
        let x = pt.position.x as f32;
        let y = pt.position.y as f32;
        let z = pt.position.z as f32;

        min_pos[0] = min_pos[0].min(x);
        min_pos[1] = min_pos[1].min(y);
        min_pos[2] = min_pos[2].min(z);

        max_pos[0] = max_pos[0].max(x);
        max_pos[1] = max_pos[1].max(y);
        max_pos[2] = max_pos[2].max(z);

        buffer_bytes.extend_from_slice(&x.to_le_bytes());
        buffer_bytes.extend_from_slice(&y.to_le_bytes());
        buffer_bytes.extend_from_slice(&z.to_le_bytes());
    }

    let pos_byte_length = count * 12;

    // Write color ubyte (VEC3 u8)
    for pt in &cloud.points {
        buffer_bytes.push(pt.color[0]);
        buffer_bytes.push(pt.color[1]);
        buffer_bytes.push(pt.color[2]);
    }

    let col_byte_length = count * 3;

    // Pad to 4-byte boundary alignment
    while buffer_bytes.len() % 4 != 0 {
        buffer_bytes.push(0);
    }

    let total_byte_length = buffer_bytes.len();
    let b64_data = base64_encode(&buffer_bytes);

    let json = format!(
        "{{\n  \"asset\": {{ \"version\": \"2.0\", \"generator\": \"Voxora Vision\" }},\n  \"scene\": 0,\n  \"scenes\": [ {{ \"nodes\": [0] }} ],\n  \"nodes\": [ {{ \"name\": \"SpatialPointCloud\", \"mesh\": 0 }} ],\n  \"meshes\": [ {{ \"primitives\": [ {{ \"attributes\": {{ \"POSITION\": 0, \"COLOR_0\": 1 }}, \"mode\": 0 }} ] }} ],\n  \"accessors\": [\n    {{\n      \"bufferView\": 0,\n      \"byteOffset\": 0,\n      \"componentType\": 5126,\n      \"count\": {count},\n      \"type\": \"VEC3\",\n      \"min\": [{:.4}, {:.4}, {:.4}],\n      \"max\": [{:.4}, {:.4}, {:.4}]\n    }},\n    {{\n      \"bufferView\": 1,\n      \"byteOffset\": 0,\n      \"componentType\": 5121,\n      \"normalized\": true,\n      \"count\": {count},\n      \"type\": \"VEC3\"\n    }}\n  ],\n  \"bufferViews\": [\n    {{\n      \"buffer\": 0,\n      \"byteOffset\": 0,\n      \"byteLength\": {pos_byte_length},\n      \"target\": 34962\n    }},\n    {{\n      \"buffer\": 0,\n      \"byteOffset\": {pos_byte_length},\n      \"byteLength\": {col_byte_length},\n      \"target\": 34962\n    }}\n  ],\n  \"buffers\": [\n    {{\n      \"uri\": \"data:application/octet-stream;base64,{b64_data}\",\n      \"byteLength\": {total_byte_length}\n    }}\n  ]\n}}",
        min_pos[0], min_pos[1], min_pos[2],
        max_pos[0], max_pos[1], max_pos[2]
    );

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

    #[test]
    fn test_export_gltf_format() {
        let mut cloud = PointCloud::new();
        cloud.push(Point3D::new(Vector3::new(1.0, 2.0, 3.0), [255, 0, 0], 1.0));
        let gltf = export_gltf_json(&cloud).unwrap();

        assert!(gltf.contains("\"asset\": { \"version\": \"2.0\""));
        assert!(gltf.contains("\"accessors\""));
        assert!(gltf.contains("\"bufferViews\""));
        assert!(gltf.contains("data:application/octet-stream;base64"));
    }
}
