//! Core types, frame structures, error definitions, and video decoding traits for Voxora.

#![warn(missing_docs)]

use std::fmt;

pub mod decoder;
pub mod stream;

pub use stream::BoundedFrameQueue;

/// Pixel format types supported by Voxora frame representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    /// 8-bit single channel Grayscale
    Grayscale,
    /// 24-bit RGB (3 channels of 8-bit unsigned integer)
    Rgb8,
    /// 32-bit RGBA (4 channels of 8-bit unsigned integer)
    Rgba8,
    /// 32-bit normalized floating point Grayscale [0.0, 1.0]
    Float32Grayscale,
}

impl PixelFormat {
    /// Returns the number of bytes per pixel for this format.
    pub fn bytes_per_pixel(&self) -> usize {
        match self {
            Self::Grayscale => 1,
            Self::Rgb8 => 3,
            Self::Rgba8 => 4,
            Self::Float32Grayscale => 4,
        }
    }
}

/// Normalized floating-point frame representation for precision image processing operations.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameF32 {
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
    /// Floating-point pixel values normalized to [0.0, 1.0]
    pub data: Vec<f32>,
}

impl FrameF32 {
    /// Creates a new floating-point frame buffer.
    pub fn new(width: u32, height: u32, data: Vec<f32>) -> Result<Self, VoxoraError> {
        let expected_size = (width as usize) * (height as usize);
        if data.len() != expected_size {
            return Err(VoxoraError::InvalidFrameDimensions {
                expected: expected_size,
                actual: data.len(),
            });
        }
        Ok(Self { width, height, data })
    }
}

/// Core internal frame representation containing image dimensions and pixel data.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Frame width in pixels
    pub width: u32,
    /// Frame height in pixels
    pub height: u32,
    /// Format of the pixel buffer
    pub format: PixelFormat,
    /// Raw pixel data buffer
    pub data: Vec<u8>,
}

impl Frame {
    /// Creates a new frame, validating that data length matches dimensions * bytes_per_pixel.
    pub fn new(
        width: u32,
        height: u32,
        format: PixelFormat,
        data: Vec<u8>,
    ) -> Result<Self, VoxoraError> {
        let expected_size = (width as usize) * (height as usize) * format.bytes_per_pixel();
        if data.len() != expected_size {
            return Err(VoxoraError::InvalidFrameDimensions {
                expected: expected_size,
                actual: data.len(),
            });
        }
        Ok(Self { width, height, format, data })
    }

    /// Provides zero-copy slice access to pixel data.
    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }

    /// Provides zero-copy mutable slice access to pixel data.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data
    }

    /// Converts the frame to single-channel Grayscale format.
    pub fn to_grayscale(&self) -> Self {
        if self.format == PixelFormat::Grayscale {
            return self.clone();
        }

        let pixel_count = (self.width * self.height) as usize;
        let mut gray_data = Vec::with_capacity(pixel_count);

        match self.format {
            PixelFormat::Grayscale => unreachable!(),
            PixelFormat::Rgb8 => {
                for chunk in self.data.chunks_exact(3) {
                    let r = chunk[0] as f32;
                    let g = chunk[1] as f32;
                    let b = chunk[2] as f32;
                    let gray = (0.299 * r + 0.587 * g + 0.114 * b) as u8;
                    gray_data.push(gray);
                }
            }
            PixelFormat::Rgba8 => {
                for chunk in self.data.chunks_exact(4) {
                    let r = chunk[0] as f32;
                    let g = chunk[1] as f32;
                    let b = chunk[2] as f32;
                    let gray = (0.299 * r + 0.587 * g + 0.114 * b) as u8;
                    gray_data.push(gray);
                }
            }
            PixelFormat::Float32Grayscale => {
                for chunk in self.data.chunks_exact(4) {
                    let val = f32::from_ne_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                    let byte_val = (val.clamp(0.0, 1.0) * 255.0) as u8;
                    gray_data.push(byte_val);
                }
            }
        }

        Self {
            width: self.width,
            height: self.height,
            format: PixelFormat::Grayscale,
            data: gray_data,
        }
    }

    /// Converts frame pixels to normalized `[0.0, 1.0]` floating point vector frame `FrameF32`.
    pub fn to_f32(&self) -> FrameF32 {
        let gray = self.to_grayscale();
        let float_data: Vec<f32> = gray.data.iter().map(|&p| p as f32 / 255.0).collect();
        FrameF32 { width: self.width, height: self.height, data: float_data }
    }

    /// Resizes the frame to new dimensions using bilinear spatial interpolation.
    pub fn resize(&self, new_width: u32, new_height: u32) -> Result<Self, VoxoraError> {
        if new_width == 0 || new_height == 0 {
            return Err(VoxoraError::InvalidFrameDimensions { expected: 1, actual: 0 });
        }

        let bpp = self.format.bytes_per_pixel();
        let mut out_data = vec![0u8; (new_width * new_height * bpp as u32) as usize];

        let x_ratio =
            if new_width > 1 { (self.width - 1) as f32 / (new_width - 1) as f32 } else { 0.0 };

        let y_ratio =
            if new_height > 1 { (self.height - 1) as f32 / (new_height - 1) as f32 } else { 0.0 };

        for y in 0..new_height {
            let src_y = y as f32 * y_ratio;
            let y_low = src_y.floor() as u32;
            let y_high = (y_low + 1).min(self.height - 1);
            let y_weight = src_y - y_low as f32;

            for x in 0..new_width {
                let src_x = x as f32 * x_ratio;
                let x_low = src_x.floor() as u32;
                let x_high = (x_low + 1).min(self.width - 1);
                let x_weight = src_x - x_low as f32;

                for c in 0..bpp {
                    let top_left =
                        self.data[(y_low * self.width + x_low) as usize * bpp + c] as f32;
                    let top_right =
                        self.data[(y_low * self.width + x_high) as usize * bpp + c] as f32;
                    let bot_left =
                        self.data[(y_high * self.width + x_low) as usize * bpp + c] as f32;
                    let bot_right =
                        self.data[(y_high * self.width + x_high) as usize * bpp + c] as f32;

                    let top = top_left * (1.0 - x_weight) + top_right * x_weight;
                    let bot = bot_left * (1.0 - x_weight) + bot_right * x_weight;
                    let val = top * (1.0 - y_weight) + bot * y_weight;

                    let out_idx = (y * new_width + x) as usize * bpp + c;
                    out_data[out_idx] = val.round().clamp(0.0, 255.0) as u8;
                }
            }
        }

        Self::new(new_width, new_height, self.format, out_data)
    }

    /// Crops a bounding box sub-region from the frame.
    pub fn crop(
        &self,
        x: u32,
        y: u32,
        crop_width: u32,
        crop_height: u32,
    ) -> Result<Self, VoxoraError> {
        if x + crop_width > self.width || y + crop_height > self.height {
            return Err(VoxoraError::InvalidFrameDimensions {
                expected: (self.width * self.height) as usize,
                actual: ((x + crop_width) * (y + crop_height)) as usize,
            });
        }

        let bpp = self.format.bytes_per_pixel();
        let mut cropped_data = Vec::with_capacity((crop_width * crop_height * bpp as u32) as usize);

        for row in y..(y + crop_height) {
            let start_idx = ((row * self.width + x) as usize) * bpp;
            let end_idx = start_idx + (crop_width as usize) * bpp;
            cropped_data.extend_from_slice(&self.data[start_idx..end_idx]);
        }

        Self::new(crop_width, crop_height, self.format, cropped_data)
    }

    /// Normalizes frame intensities so that minimum pixel value becomes 0 and maximum becomes 255.
    pub fn normalize(&self) -> Self {
        if self.data.is_empty() {
            return self.clone();
        }

        let mut min_val = u8::MAX;
        let mut max_val = u8::MIN;

        for &val in &self.data {
            if val < min_val {
                min_val = val;
            }
            if val > max_val {
                max_val = val;
            }
        }

        if min_val == max_val {
            return self.clone();
        }

        let range = (max_val - min_val) as f32;
        let normalized_data =
            self.data.iter().map(|&v| (((v - min_val) as f32 / range) * 255.0) as u8).collect();

        Self { width: self.width, height: self.height, format: self.format, data: normalized_data }
    }
}

/// Voxora error types for failure modes across decoding, processing, geometry, and rendering.
#[derive(Debug, PartialEq)]
pub enum VoxoraError {
    /// Invalid frame dimension or buffer mismatch
    InvalidFrameDimensions {
        /// Expected size in bytes
        expected: usize,
        /// Actual buffer size provided
        actual: usize,
    },
    /// Feature tracking or matching failure
    TrackingFailed(String),
    /// Geometric estimation error
    GeometryError(String),
    /// Video decoding error
    DecoderError(String),
}

impl fmt::Display for VoxoraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFrameDimensions { expected, actual } => {
                write!(
                    f,
                    "Invalid frame dimensions: expected buffer of {expected} bytes, got {actual}"
                )
            }
            Self::TrackingFailed(msg) => write!(f, "Tracking failed: {msg}"),
            Self::GeometryError(msg) => write!(f, "Geometry error: {msg}"),
            Self::DecoderError(msg) => write!(f, "Decoder error: {msg}"),
        }
    }
}

impl std::error::Error for VoxoraError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_frame_creation() {
        let data = vec![0u8; 100 * 100 * 3];
        let frame = Frame::new(100, 100, PixelFormat::Rgb8, data);
        assert!(frame.is_ok());
    }

    #[test]
    fn test_invalid_frame_creation() {
        let data = vec![0u8; 50];
        let frame = Frame::new(100, 100, PixelFormat::Rgb8, data);
        assert!(frame.is_err());
    }

    #[test]
    fn test_rgb_to_grayscale_conversion() {
        let rgb_data = vec![255, 0, 0, 0, 255, 0]; // Red pixel, Green pixel
        let frame = Frame::new(2, 1, PixelFormat::Rgb8, rgb_data).unwrap();
        let gray = frame.to_grayscale();

        assert_eq!(gray.format, PixelFormat::Grayscale);
        assert_eq!(gray.width, 2);
        assert_eq!(gray.height, 1);
        assert_eq!(gray.data.len(), 2);
        // Red luma = 0.299 * 255 = ~76
        assert_eq!(gray.data[0], 76);
        // Green luma = 0.587 * 255 = ~149
        assert_eq!(gray.data[1], 149);
    }

    #[test]
    fn test_frame_resize() {
        let data = vec![0, 100, 200, 255]; // 2x2 grayscale
        let frame = Frame::new(2, 2, PixelFormat::Grayscale, data).unwrap();
        let resized = frame.resize(4, 4).unwrap();

        assert_eq!(resized.width, 4);
        assert_eq!(resized.height, 4);
        assert_eq!(resized.data.len(), 16);
    }

    #[test]
    fn test_frame_crop() {
        let data = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let frame = Frame::new(4, 4, PixelFormat::Grayscale, data).unwrap();
        let cropped = frame.crop(1, 1, 2, 2).unwrap();

        assert_eq!(cropped.width, 2);
        assert_eq!(cropped.height, 2);
        assert_eq!(cropped.data, vec![6, 7, 10, 11]);
    }

    #[test]
    fn test_frame_f32_conversion() {
        let data = vec![0, 127, 255];
        let frame = Frame::new(3, 1, PixelFormat::Grayscale, data).unwrap();
        let f32_frame = frame.to_f32();

        assert_eq!(f32_frame.width, 3);
        assert_eq!(f32_frame.height, 1);
        assert!((f32_frame.data[0] - 0.0).abs() < 1e-3);
        assert!((f32_frame.data[1] - 0.498).abs() < 1e-2);
        assert!((f32_frame.data[2] - 1.0).abs() < 1e-3);
    }
}
