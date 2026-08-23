//! Video decoding traits, metadata structures, streaming frame readers, and decoder backends.

use crate::{Frame, PixelFormat, VoxoraError};
use std::collections::VecDeque;
use std::io::Read;
use std::process::{Child, Command, Stdio};

/// Color space metadata for decoded video streams.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorSpace {
    /// Standard RGB color space (sRGB)
    SRgb,
    /// Linear RGB color space
    LinearRgb,
    /// Unknown or unspecified color space
    Unknown,
}

/// Metadata extracted from a video input stream.
#[derive(Debug, Clone, PartialEq)]
pub struct VideoMetadata {
    /// Frame width in pixels
    pub width: u32,
    /// Frame height in pixels
    pub height: u32,
    /// Frames per second (FPS)
    pub fps: f64,
    /// Total frame count (if known or seekable)
    pub total_frames: Option<usize>,
    /// Duration of the video in seconds (if known)
    pub duration_secs: Option<f64>,
    /// Pixel format of decoded frames
    pub pixel_format: PixelFormat,
    /// Color space of the video stream
    pub color_space: ColorSpace,
}

/// Pluggable video decoder interface.
pub trait VideoDecoder {
    /// Returns reference to metadata of the video.
    fn metadata(&self) -> &VideoMetadata;

    /// Decodes and returns the next frame, or None if end of stream is reached.
    fn next_frame(&mut self) -> Result<Option<Frame>, VoxoraError>;

    /// Seeks to a specific frame index if supported by the backend.
    fn seek(&mut self, frame_index: usize) -> Result<(), VoxoraError>;
}

/// Streaming video reader wrapping a decoder with configurable frame buffering.
#[derive(Debug)]
pub struct VideoReader<D: VideoDecoder> {
    decoder: D,
    buffer: VecDeque<Frame>,
    buffer_capacity: usize,
    current_frame_index: usize,
}

impl<D: VideoDecoder> VideoReader<D> {
    /// Creates a new streaming VideoReader with specified frame buffer capacity.
    pub fn new(decoder: D, buffer_capacity: usize) -> Self {
        let capacity = buffer_capacity.max(1);
        Self {
            decoder,
            buffer: VecDeque::with_capacity(capacity),
            buffer_capacity: capacity,
            current_frame_index: 0,
        }
    }

    /// Returns metadata of the underlying video.
    pub fn metadata(&self) -> &VideoMetadata {
        self.decoder.metadata()
    }

    /// Returns current frame index read from stream.
    pub fn current_frame_index(&self) -> usize {
        self.current_frame_index
    }

    /// Reads next frame from stream using internal buffer queue.
    pub fn next_frame(&mut self) -> Result<Option<Frame>, VoxoraError> {
        if self.buffer.is_empty() {
            // Fill buffer up to capacity
            while self.buffer.len() < self.buffer_capacity {
                match self.decoder.next_frame()? {
                    Some(frame) => self.buffer.push_back(frame),
                    None => break,
                }
            }
        }

        if let Some(frame) = self.buffer.pop_front() {
            self.current_frame_index += 1;
            Ok(Some(frame))
        } else {
            Ok(None)
        }
    }
}

impl<D: VideoDecoder> Iterator for VideoReader<D> {
    type Item = Result<Frame, VoxoraError>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.next_frame() {
            Ok(Some(frame)) => Some(Ok(frame)),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    }
}

/// Synthetic video decoder generating deterministic synthetic frames for testing geometry & motion.
#[derive(Debug)]
pub struct SyntheticVideoDecoder {
    metadata: VideoMetadata,
    current_frame: usize,
    max_frames: usize,
}

impl SyntheticVideoDecoder {
    /// Creates a synthetic video decoder emitting `max_frames` at specified dimensions.
    pub fn new(width: u32, height: u32, max_frames: usize, fps: f64) -> Self {
        let metadata = VideoMetadata {
            width,
            height,
            fps,
            total_frames: Some(max_frames),
            duration_secs: Some(max_frames as f64 / fps),
            pixel_format: PixelFormat::Rgb8,
            color_space: ColorSpace::SRgb,
        };

        Self { metadata, current_frame: 0, max_frames }
    }
}

impl VideoDecoder for SyntheticVideoDecoder {
    fn metadata(&self) -> &VideoMetadata {
        &self.metadata
    }

    fn next_frame(&mut self) -> Result<Option<Frame>, VoxoraError> {
        if self.current_frame >= self.max_frames {
            return Ok(None);
        }

        let w = self.metadata.width;
        let h = self.metadata.height;
        let frame_idx = self.current_frame;
        self.current_frame += 1;

        let mut data = vec![0u8; (w * h * 3) as usize];
        let offset = (frame_idx * 5) % (w as usize);

        for y in 0..h as usize {
            for x in 0..w as usize {
                let idx = (y * (w as usize) + x) * 3;
                let val = (((x + offset) ^ y) & 0xFF) as u8;
                data[idx] = val;
                data[idx + 1] = ((x * 2) & 0xFF) as u8;
                data[idx + 2] = ((y * 2) & 0xFF) as u8;
            }
        }

        Frame::new(w, h, PixelFormat::Rgb8, data).map(Some)
    }

    fn seek(&mut self, frame_index: usize) -> Result<(), VoxoraError> {
        if frame_index >= self.max_frames {
            return Err(VoxoraError::DecoderError(format!(
                "Seek target {} out of bounds (max {})",
                frame_index, self.max_frames
            )));
        }
        self.current_frame = frame_index;
        Ok(())
    }
}

/// Image sequence video decoder reading pre-existing frames from an in-memory frame list.
#[derive(Debug)]
pub struct ImageSequenceDecoder {
    metadata: VideoMetadata,
    frames: Vec<Frame>,
    current_frame: usize,
}

impl ImageSequenceDecoder {
    /// Creates an image sequence decoder from a list of frames.
    pub fn new(frames: Vec<Frame>, fps: f64) -> Result<Self, VoxoraError> {
        if frames.is_empty() {
            return Err(VoxoraError::DecoderError(
                "Cannot create ImageSequenceDecoder with empty frame list".into(),
            ));
        }

        let first = &frames[0];
        let metadata = VideoMetadata {
            width: first.width,
            height: first.height,
            fps,
            total_frames: Some(frames.len()),
            duration_secs: Some(frames.len() as f64 / fps),
            pixel_format: first.format,
            color_space: ColorSpace::SRgb,
        };

        Ok(Self { metadata, frames, current_frame: 0 })
    }
}

impl VideoDecoder for ImageSequenceDecoder {
    fn metadata(&self) -> &VideoMetadata {
        &self.metadata
    }

    fn next_frame(&mut self) -> Result<Option<Frame>, VoxoraError> {
        if self.current_frame >= self.frames.len() {
            return Ok(None);
        }
        let frame = self.frames[self.current_frame].clone();
        self.current_frame += 1;
        Ok(Some(frame))
    }

    fn seek(&mut self, frame_index: usize) -> Result<(), VoxoraError> {
        if frame_index >= self.frames.len() {
            return Err(VoxoraError::DecoderError(format!(
                "Seek index {} out of range",
                frame_index
            )));
        }
        self.current_frame = frame_index;
        Ok(())
    }
}

/// Video decoder backed by an external FFmpeg process streaming raw RGB24 frames via pipe.
pub struct FffmpegVideoDecoder {
    metadata: VideoMetadata,
    stdout: std::process::ChildStdout,
    _child: Child,
    frame_bytes: usize,
    current_frame: usize,
}

impl FffmpegVideoDecoder {
    /// Spawns FFmpeg to decode the specified video path into raw RGB24 frames scaled to `(width, height)`.
    pub fn open(path: &str, width: u32, height: u32) -> Result<Self, VoxoraError> {
        let scale_filter = format!("scale={}:{}", width, height);
        let mut child = Command::new("ffmpeg")
            .args([
                "-i",
                path,
                "-vf",
                &scale_filter,
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-loglevel",
                "quiet",
                "-",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|e| {
                VoxoraError::DecoderError(format!("Failed to launch ffmpeg for video path '{}': {}", path, e))
            })?;

        let stdout = child.stdout.take().ok_or_else(|| {
            VoxoraError::DecoderError("Failed to open ffmpeg stdout stream pipe".into())
        })?;

        let frame_bytes = (width * height * 3) as usize;
        let metadata = VideoMetadata {
            width,
            height,
            fps: 30.0,
            total_frames: None,
            duration_secs: None,
            pixel_format: PixelFormat::Rgb8,
            color_space: ColorSpace::SRgb,
        };

        Ok(Self {
            metadata,
            stdout,
            _child: child,
            frame_bytes,
            current_frame: 0,
        })
    }
}

impl VideoDecoder for FffmpegVideoDecoder {
    fn metadata(&self) -> &VideoMetadata {
        &self.metadata
    }

    fn next_frame(&mut self) -> Result<Option<Frame>, VoxoraError> {
        let mut buffer = vec![0u8; self.frame_bytes];
        let mut bytes_read = 0;

        while bytes_read < self.frame_bytes {
            match self.stdout.read(&mut buffer[bytes_read..]) {
                Ok(0) => break, // EOF reached
                Ok(n) => bytes_read += n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    return Err(VoxoraError::DecoderError(format!(
                        "Error reading frame stream from ffmpeg: {}",
                        e
                    )))
                }
            }
        }

        if bytes_read < self.frame_bytes {
            return Ok(None);
        }

        self.current_frame += 1;
        Frame::new(self.metadata.width, self.metadata.height, PixelFormat::Rgb8, buffer).map(Some)
    }

    fn seek(&mut self, _frame_index: usize) -> Result<(), VoxoraError> {
        Err(VoxoraError::DecoderError(
            "Stream seeking not supported on live FFmpeg stdout pipe".into(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_synthetic_video_decoder() {
        let mut decoder = SyntheticVideoDecoder::new(64, 64, 10, 30.0);
        assert_eq!(decoder.metadata().total_frames, Some(10));

        let frame1 = decoder.next_frame().unwrap();
        assert!(frame1.is_some());
        let f = frame1.unwrap();
        assert_eq!(f.width, 64);
        assert_eq!(f.height, 64);
        assert_eq!(f.format, PixelFormat::Rgb8);
    }

    #[test]
    fn test_video_reader_streaming() {
        let decoder = SyntheticVideoDecoder::new(32, 32, 5, 30.0);
        let mut reader = VideoReader::new(decoder, 2);

        let mut count = 0;
        while let Ok(Some(_frame)) = reader.next_frame() {
            count += 1;
        }

        assert_eq!(count, 5);
        assert_eq!(reader.current_frame_index(), 5);
    }

    #[test]
    fn test_image_sequence_decoder() {
        let f1 = Frame::new(10, 10, PixelFormat::Grayscale, vec![0; 100]).unwrap();
        let f2 = Frame::new(10, 10, PixelFormat::Grayscale, vec![255; 100]).unwrap();

        let mut decoder = ImageSequenceDecoder::new(vec![f1, f2], 24.0).unwrap();
        assert_eq!(decoder.metadata().total_frames, Some(2));

        let read1 = decoder.next_frame().unwrap().unwrap();
        assert_eq!(read1.data[0], 0);

        let read2 = decoder.next_frame().unwrap().unwrap();
        assert_eq!(read2.data[0], 255);

        assert!(decoder.next_frame().unwrap().is_none());
    }
}
