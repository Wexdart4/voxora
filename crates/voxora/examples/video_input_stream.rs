//! Example demonstrating streaming video frame decoding and frame manipulation pipeline.

use voxora::{SyntheticVideoDecoder, VideoDecoder, VideoReader};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Voxora Streaming Video Reader Example ---");

    // 1. Instantiate a synthetic video stream (128x128 @ 30 FPS, 15 frames)
    let decoder = SyntheticVideoDecoder::new(128, 128, 15, 30.0);
    println!("Stream Metadata:\n  {:?}", decoder.metadata());

    // 2. Wrap decoder in a streaming VideoReader with a buffer queue capacity of 3
    let mut reader = VideoReader::new(decoder, 3);

    // 3. Stream frames sequentially without loading entire stream into memory
    while let Some(frame) = reader.next_frame()? {
        let frame_idx = reader.current_frame_index();
        let gray_frame = frame.to_grayscale();
        let resized_frame = gray_frame.resize(64, 64)?;
        let f32_frame = resized_frame.to_f32();

        println!(
            "Frame #{:<2} | Original: {}x{} | Resized Grayscale: {}x{} | F32 Min/Max Sample: {:.2}/{:.2}",
            frame_idx,
            frame.width,
            frame.height,
            resized_frame.width,
            resized_frame.height,
            f32_frame.data[0],
            f32_frame.data[f32_frame.data.len() - 1]
        );
    }

    println!("--- Streaming Video Processing Completed ---");
    Ok(())
}
