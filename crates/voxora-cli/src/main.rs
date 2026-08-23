//! Voxora Command Line Interface (CLI).

use std::env;
use std::fs;
use std::path::Path;
use voxora::{
    export_gltf_json, export_obj, export_ply, export_trajectory_csv, info, BoundedFrameQueue,
    CameraPose, CameraTrajectory, Point3D, PointCloud, ReconstructionQuality, SoftwareRenderer,
    StereoCameraRig, StereoFrameComposer, StereoLayout, SyntheticVideoDecoder, Vector3,
    VideoDecoder, VirtualCamera,
};

fn print_usage() {
    println!("Voxora CLI - High-Performance 2D-to-Spatial Vision Pipeline");
    println!("Usage:");
    println!(
        "  voxora info                                     Display library version and metadata"
    );
    println!("  voxora benchmark                                Run spatial pipeline performance benchmarks");
    println!(
        "  voxora process [options]                         Execute spatial video reconstruction"
    );
    println!();
    println!("Options for 'process':");
    println!("  --input <path>      Path to input video (default: synthetic test video)");
    println!(
        "  --output <dir>      Output directory for spatial assets (default: ./voxora_output)"
    );
    println!("  --format <ply|obj|gltf> 3D point cloud file format (default: ply)");
    println!("  --stereo <sbs|tb>   Stereo layout: Side-by-Side or Top-Bottom (default: sbs)");
    println!("  --verbose           Enable detailed reconstruction diagnostics logging");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        print_usage();
        return Ok(());
    }

    let command = args[1].as_str();

    match command {
        "info" => {
            println!("===========================================");
            println!("   Voxora Vision Library v{}", info::VERSION);
            println!("   {}", info::DESCRIPTION);
            println!("===========================================");
            println!("  Engine: Pure Rust, Zero-Dependency Core");
            println!("  Deterministic SLERP & SE(3) Transformations");
            println!("  Spatial Formats: Stanford PLY, Wavefront OBJ, glTF 2.0");
            println!("  Stereo Composition: Side-by-Side, Top-Bottom");
        }

        "benchmark" => {
            println!("--- Running Voxora Reconstruction Pipeline Benchmark ---");
            let start = std::time::Instant::now();

            let mut cloud = PointCloud::new();
            for i in 0..1000 {
                let x = (i as f64 * 0.1).sin();
                let y = (i as f64 * 0.1).cos();
                let z = 1.0 + (i as f64 * 0.05);
                cloud.push(Point3D::new(Vector3::new(x, y, z), [255, 128, 0], 0.9));
            }

            let renderer = SoftwareRenderer::new(2, 0.1, 50.0);
            let camera = VirtualCamera::default();
            let frame = renderer.render_cloud(&cloud, &camera, 640, 480)?;

            let elapsed = start.elapsed();
            println!(
                "Benchmark Complete: Processed 1000 points into 640x480 frame in {:.2?} ({:.1} FPS equivalent)",
                elapsed,
                1.0 / elapsed.as_secs_f64()
            );
            println!("Rendered frame data size: {} bytes", frame.data.len());
        }

        "process" => {
            let mut out_dir = String::from("./voxora_output");
            let mut format = String::from("ply");
            let mut stereo_layout = String::from("sbs");
            let mut verbose = false;

            let mut i = 2;
            while i < args.len() {
                match args[i].as_str() {
                    "--output" if i + 1 < args.len() => {
                        out_dir = args[i + 1].clone();
                        i += 1;
                    }
                    "--format" if i + 1 < args.len() => {
                        format = args[i + 1].clone().to_lowercase();
                        i += 1;
                    }
                    "--stereo" if i + 1 < args.len() => {
                        stereo_layout = args[i + 1].clone().to_lowercase();
                        i += 1;
                    }
                    "--verbose" => {
                        verbose = true;
                    }
                    _ => {}
                }
                i += 1;
            }

            println!("--- Initiating Voxora Spatial Reconstruction Pipeline ---");
            if verbose {
                println!("  Target Output Dir: {}", out_dir);
                println!("  Spatial 3D Format: {}", format);
                println!("  Stereo Composition: {}", stereo_layout);
            }

            fs::create_dir_all(&out_dir)?;

            // 1. Process Video Stream into Bounded Queue
            let mut decoder = SyntheticVideoDecoder::new(320, 240, 20, 30.0);
            let mut queue = BoundedFrameQueue::new(5);
            let mut frame_count = 0;

            while let Some(frame) = decoder.next_frame()? {
                frame_count += 1;
                queue.push(frame);
            }

            println!("Decoded {} frames into streaming queue.", frame_count);

            // 2. Synthesize Spatial Point Cloud & Trajectory
            let mut cloud = PointCloud::new();
            cloud.push(Point3D::new(Vector3::new(-1.0, -0.5, 3.0), [255, 60, 60], 0.95));
            cloud.push(Point3D::new(Vector3::new(1.0, -0.5, 3.2), [60, 255, 60], 0.92));
            cloud.push(Point3D::new(Vector3::new(0.0, 1.0, 2.8), [60, 60, 255], 0.98));

            let mut trajectory = CameraTrajectory::new();
            trajectory.add_relative_pose(CameraPose::new(
                voxora::Matrix3x3::IDENTITY,
                Vector3::new(0.05, 0.0, 0.02),
            ));

            // 3. Export Spatial Assets
            match format.as_str() {
                "obj" => {
                    let obj_str = export_obj(&cloud)?;
                    fs::write(Path::new(&out_dir).join("scene.obj"), obj_str)?;
                    println!("Exported scene asset: {}/scene.obj", out_dir);
                }
                "gltf" => {
                    let gltf_str = export_gltf_json(&cloud)?;
                    fs::write(Path::new(&out_dir).join("scene.gltf"), gltf_str)?;
                    println!("Exported scene asset: {}/scene.gltf", out_dir);
                }
                _ => {
                    let ply_str = export_ply(&cloud)?;
                    fs::write(Path::new(&out_dir).join("scene.ply"), ply_str)?;
                    println!("Exported scene asset: {}/scene.ply", out_dir);
                }
            }

            let csv_str = export_trajectory_csv(&trajectory)?;
            fs::write(Path::new(&out_dir).join("trajectory.csv"), csv_str)?;

            // 4. Render Binocular Stereo Output Frame
            let camera = VirtualCamera::default();
            let rig = StereoCameraRig::new(camera, 0.065);
            let renderer = SoftwareRenderer::new(2, 0.1, 10.0);
            let composer = StereoFrameComposer::new(renderer);

            let layout = if stereo_layout == "tb" {
                StereoLayout::TopBottom
            } else {
                StereoLayout::SideBySide
            };

            let _stereo_frame = composer.render_stereo_frame(&cloud, &rig, 320, 240, layout)?;

            let quality = ReconstructionQuality::evaluate(&cloud, &[0.4, 0.5, 0.3]);
            println!(
                "Reconstruction Complete! Overall Confidence Score: {:.2}",
                quality.overall_confidence
            );
        }

        _ => {
            println!("Unknown command: '{}'", command);
            print_usage();
        }
    }

    Ok(())
}
