//! Example demonstrating software 3D rendering, PLY/OBJ/glTF/CSV file exports, and Side-by-Side stereo frame generation.

use voxora::{
    export_gltf_json, export_obj, export_ply, export_trajectory_csv, CameraPose, CameraTrajectory,
    Point3D, PointCloud, SoftwareRenderer, StereoCameraRig, StereoFrameComposer, StereoLayout,
    Vector3, VirtualCamera,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora 3D Rendering, Spatial Exporters & Stereo Video Demo ---");

    // 1. Construct 3D Spatial Point Cloud
    let mut cloud = PointCloud::new();
    cloud.push(Point3D::new(Vector3::new(-0.5, -0.5, 2.0), [255, 0, 0], 1.0));
    cloud.push(Point3D::new(Vector3::new(0.5, -0.5, 2.0), [0, 255, 0], 1.0));
    cloud.push(Point3D::new(Vector3::new(0.0, 0.5, 2.5), [0, 0, 255], 1.0));
    cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 1.8), [255, 255, 0], 1.0));

    println!("Constructed 3D Point Cloud with {} spatial points.", cloud.len());

    // 2. Export Spatial Assets (PLY, OBJ, glTF JSON)
    let ply_str = export_ply(&cloud)?;
    let obj_str = export_obj(&cloud)?;
    let gltf_str = export_gltf_json(&cloud)?;

    println!("Exported Stanford PLY Asset (ASCII): {} bytes", ply_str.len());
    println!("Exported Wavefront OBJ Asset: {} bytes", obj_str.len());
    println!("Exported glTF 2.0 JSON Structure: {} bytes", gltf_str.len());

    // 3. Export Camera Trajectory CSV
    let mut trajectory = CameraTrajectory::new();
    trajectory.add_relative_pose(CameraPose::new(
        voxora::Matrix3x3::IDENTITY,
        Vector3::new(0.1, 0.0, 0.05),
    ));
    let csv_str = export_trajectory_csv(&trajectory)?;

    println!(
        "Exported Camera Trajectory CSV: {} bytes, {} poses",
        csv_str.len(),
        trajectory.poses.len()
    );

    // 4. Software CPU 3D Z-Buffer Render
    let virtual_cam = VirtualCamera::default();
    let renderer = SoftwareRenderer::new(3, 0.1, 10.0);
    let rendered_frame = renderer.render_cloud(&cloud, &virtual_cam, 128, 128)?;

    println!(
        "Software 3D Renderer: Generated 2D RGB Frame ({}x{} pixels, buffer size {} bytes)",
        rendered_frame.width,
        rendered_frame.height,
        rendered_frame.data.len()
    );

    // 5. Render Binocular Stereo Side-by-Side (SBS) Frame
    let rig = StereoCameraRig::new(virtual_cam, 0.065); // 65mm interocular baseline
    let composer = StereoFrameComposer::new(renderer);

    let sbs_frame =
        composer.render_stereo_frame(&cloud, &rig, 128, 128, StereoLayout::SideBySide)?;

    println!(
        "Stereo Frame Composer: Rendered Side-by-Side (SBS) Stereo Frame ({}x{} pixels, buffer size {} bytes)",
        sbs_frame.width, sbs_frame.height, sbs_frame.data.len()
    );

    println!("--- 3D Rendering & Spatial Output Completed ---");
    Ok(())
}
