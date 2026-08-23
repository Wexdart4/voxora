//! Image filtering, normalization, histogram processing, and Gaussian pyramids.

use voxora_core::{Frame, PixelFormat, VoxoraError};

/// Applies a 2D Gaussian blur filter to a grayscale or RGB frame.
pub fn gaussian_blur(frame: &Frame, kernel_size: usize, sigma: f32) -> Result<Frame, VoxoraError> {
    if kernel_size % 2 == 0 || kernel_size == 0 {
        return Err(VoxoraError::GeometryError(
            "Kernel size for Gaussian blur must be odd and positive".into(),
        ));
    }

    let gray = frame.to_grayscale();
    let width = gray.width as usize;
    let height = gray.height as usize;
    let radius = kernel_size / 2;

    // 1D Gaussian kernel computation
    let mut kernel = vec![0.0f32; kernel_size];
    let mut sum = 0.0f32;
    let sigma_sq = sigma * sigma;

    for (i, elem) in kernel.iter_mut().enumerate() {
        let x = i as f32 - radius as f32;
        let val = (-x * x / (2.0 * sigma_sq)).exp();
        *elem = val;
        sum += val;
    }
    for k in &mut kernel {
        *k /= sum;
    }

    // Horizontal pass
    let mut temp = vec![0.0f32; width * height];
    for y in 0..height {
        for x in 0..width {
            let mut acc = 0.0f32;
            for (k, &k_val) in kernel.iter().enumerate() {
                let k_offset = k as i32 - radius as i32;
                let src_x = (x as i32 + k_offset).clamp(0, width as i32 - 1) as usize;
                acc += gray.data[y * width + src_x] as f32 * k_val;
            }
            temp[y * width + x] = acc;
        }
    }

    // Vertical pass
    let mut out_data = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            let mut acc = 0.0f32;
            for (k, &k_val) in kernel.iter().enumerate() {
                let k_offset = k as i32 - radius as i32;
                let src_y = (y as i32 + k_offset).clamp(0, height as i32 - 1) as usize;
                acc += temp[src_y * width + x] * k_val;
            }
            out_data[y * width + x] = acc.round().clamp(0.0, 255.0) as u8;
        }
    }

    Frame::new(gray.width, gray.height, PixelFormat::Grayscale, out_data)
}

/// Applies a box averaging filter to a grayscale frame.
pub fn box_filter(frame: &Frame, kernel_size: usize) -> Result<Frame, VoxoraError> {
    if kernel_size % 2 == 0 || kernel_size == 0 {
        return Err(VoxoraError::GeometryError(
            "Kernel size for box filter must be odd and positive".into(),
        ));
    }

    let gray = frame.to_grayscale();
    let width = gray.width as usize;
    let height = gray.height as usize;
    let radius = kernel_size / 2;
    let area = (kernel_size * kernel_size) as f32;

    let mut out_data = vec![0u8; width * height];

    for y in 0..height {
        for x in 0..width {
            let mut sum = 0.0f32;
            for ky in 0..kernel_size {
                let src_y =
                    (y as i32 + ky as i32 - radius as i32).clamp(0, height as i32 - 1) as usize;
                for kx in 0..kernel_size {
                    let src_x =
                        (x as i32 + kx as i32 - radius as i32).clamp(0, width as i32 - 1) as usize;
                    sum += gray.data[src_y * width + src_x] as f32;
                }
            }
            out_data[y * width + x] = (sum / area).round().clamp(0.0, 255.0) as u8;
        }
    }

    Frame::new(gray.width, gray.height, PixelFormat::Grayscale, out_data)
}

/// Applies a median noise-reduction filter.
pub fn median_filter(frame: &Frame, kernel_size: usize) -> Result<Frame, VoxoraError> {
    if kernel_size % 2 == 0 || kernel_size == 0 {
        return Err(VoxoraError::GeometryError(
            "Kernel size for median filter must be odd and positive".into(),
        ));
    }

    let gray = frame.to_grayscale();
    let width = gray.width as usize;
    let height = gray.height as usize;
    let radius = kernel_size / 2;
    let mut window = vec![0u8; kernel_size * kernel_size];

    let mut out_data = vec![0u8; width * height];

    for y in 0..height {
        for x in 0..width {
            let mut count = 0;
            for ky in 0..kernel_size {
                let src_y =
                    (y as i32 + ky as i32 - radius as i32).clamp(0, height as i32 - 1) as usize;
                for kx in 0..kernel_size {
                    let src_x =
                        (x as i32 + kx as i32 - radius as i32).clamp(0, width as i32 - 1) as usize;
                    window[count] = gray.data[src_y * width + src_x];
                    count += 1;
                }
            }
            window.sort_unstable();
            out_data[y * width + x] = window[window.len() / 2];
        }
    }

    Frame::new(gray.width, gray.height, PixelFormat::Grayscale, out_data)
}

/// Adjusts image contrast (alpha) and brightness (beta).
pub fn contrast_brightness_normalize(frame: &Frame, alpha: f32, beta: f32) -> Frame {
    let mut out = frame.clone();
    for pixel in &mut out.data {
        let val = (*pixel as f32) * alpha + beta;
        *pixel = val.clamp(0.0, 255.0) as u8;
    }
    out
}

/// Computes the 256-bin intensity histogram of a grayscale frame.
pub fn histogram(frame: &Frame) -> [u32; 256] {
    let gray = frame.to_grayscale();
    let mut hist = [0u32; 256];
    for &pixel in &gray.data {
        hist[pixel as usize] += 1;
    }
    hist
}

/// Performs global histogram equalization to enhance image contrast.
pub fn histogram_equalization(frame: &Frame) -> Frame {
    let gray = frame.to_grayscale();
    let hist = histogram(&gray);
    let total_pixels = gray.data.len() as f32;

    // Cumulative distribution function (CDF)
    let mut cdf = [0.0f32; 256];
    let mut acc = 0u32;
    for i in 0..256 {
        acc += hist[i];
        cdf[i] = acc as f32 / total_pixels;
    }

    let mut out_data = vec![0u8; gray.data.len()];
    for (i, &pixel) in gray.data.iter().enumerate() {
        out_data[i] = (cdf[pixel as usize] * 255.0).round() as u8;
    }

    Frame::new(gray.width, gray.height, PixelFormat::Grayscale, out_data).unwrap()
}

/// Gaussian Image Pyramid for multi-scale feature detection.
#[derive(Debug, Clone)]
pub struct GaussianPyramid {
    /// Pyramid levels starting from level 0 (original scale) down to coarsest level
    pub levels: Vec<Frame>,
}

impl GaussianPyramid {
    /// Constructs a Gaussian pyramid with `num_levels` downsampled octave scales.
    pub fn build(frame: &Frame, num_levels: usize) -> Result<Self, VoxoraError> {
        let mut levels = Vec::with_capacity(num_levels.max(1));
        let mut current = frame.to_grayscale();
        levels.push(current.clone());

        for _ in 1..num_levels {
            if current.width < 4 || current.height < 4 {
                break;
            }
            let blurred = gaussian_blur(&current, 3, 1.0)?;
            let new_w = (current.width / 2).max(1);
            let new_h = (current.height / 2).max(1);
            current = blurred.resize(new_w, new_h)?;
            levels.push(current.clone());
        }

        Ok(Self { levels })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaussian_blur() {
        let frame = Frame::new(10, 10, PixelFormat::Grayscale, vec![100; 100]).unwrap();
        let blurred = gaussian_blur(&frame, 3, 1.0).unwrap();
        assert_eq!(blurred.width, 10);
        assert_eq!(blurred.height, 10);
        assert_eq!(blurred.data[0], 100);
    }

    #[test]
    fn test_histogram_equalization() {
        let frame = Frame::new(
            4,
            4,
            PixelFormat::Grayscale,
            vec![10, 50, 100, 200, 10, 50, 100, 200, 10, 50, 100, 200, 10, 50, 100, 200],
        )
        .unwrap();
        let eq = histogram_equalization(&frame);
        assert_eq!(eq.width, 4);
        assert_eq!(eq.height, 4);
    }

    #[test]
    fn test_gaussian_pyramid_build() {
        let frame = Frame::new(32, 32, PixelFormat::Grayscale, vec![128; 1024]).unwrap();
        let pyramid = GaussianPyramid::build(&frame, 3).unwrap();
        assert_eq!(pyramid.levels.len(), 3);
        assert_eq!(pyramid.levels[0].width, 32);
        assert_eq!(pyramid.levels[1].width, 16);
        assert_eq!(pyramid.levels[2].width, 8);
    }
}
