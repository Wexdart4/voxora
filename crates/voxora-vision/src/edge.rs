//! Image edge detection, gradient operators (Sobel, Scharr), and Canny edge detector.

use crate::filter::gaussian_blur;
use voxora_core::{Frame, PixelFormat, VoxoraError};

/// Container for horizontal/vertical image gradients, magnitude, and orientation angle.
#[derive(Debug, Clone)]
pub struct GradientMap {
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
    /// Horizontal gradient Gx values
    pub gx: Vec<f32>,
    /// Vertical gradient Gy values
    pub gy: Vec<f32>,
    /// Gradient magnitude |G|
    pub magnitude: Vec<f32>,
    /// Gradient orientation angle in radians [-PI, PI]
    pub orientation: Vec<f32>,
}

/// Computes Sobel horizontal and vertical gradients ($G_x, G_y$).
pub fn sobel_operator(frame: &Frame) -> Result<GradientMap, VoxoraError> {
    let gray = frame.to_grayscale();
    let width = gray.width as usize;
    let height = gray.height as usize;

    let mut gx = vec![0.0f32; width * height];
    let mut gy = vec![0.0f32; width * height];
    let mut magnitude = vec![0.0f32; width * height];
    let mut orientation = vec![0.0f32; width * height];

    for y in 1..height - 1 {
        for x in 1..width - 1 {
            let p00 = gray.data[(y - 1) * width + (x - 1)] as f32;
            let p01 = gray.data[(y - 1) * width + x] as f32;
            let p02 = gray.data[(y - 1) * width + (x + 1)] as f32;

            let p10 = gray.data[y * width + (x - 1)] as f32;
            let p12 = gray.data[y * width + (x + 1)] as f32;

            let p20 = gray.data[(y + 1) * width + (x - 1)] as f32;
            let p21 = gray.data[(y + 1) * width + x] as f32;
            let p22 = gray.data[(y + 1) * width + (x + 1)] as f32;

            // Sobel kernels:
            // Gx = [-1 0 1; -2 0 2; -1 0 1]
            // Gy = [-1 -2 -1; 0 0 0; 1 2 1]
            let val_gx = (p02 + 2.0 * p12 + p22) - (p00 + 2.0 * p10 + p20);
            let val_gy = (p20 + 2.0 * p21 + p22) - (p00 + 2.0 * p01 + p02);

            let idx = y * width + x;
            gx[idx] = val_gx;
            gy[idx] = val_gy;
            let mag = (val_gx * val_gx + val_gy * val_gy).sqrt();
            magnitude[idx] = mag;
            orientation[idx] = val_gy.atan2(val_gx);
        }
    }

    Ok(GradientMap { width: gray.width, height: gray.height, gx, gy, magnitude, orientation })
}

/// Computes Scharr horizontal and vertical gradients (higher rotational symmetry).
pub fn scharr_operator(frame: &Frame) -> Result<GradientMap, VoxoraError> {
    let gray = frame.to_grayscale();
    let width = gray.width as usize;
    let height = gray.height as usize;

    let mut gx = vec![0.0f32; width * height];
    let mut gy = vec![0.0f32; width * height];
    let mut magnitude = vec![0.0f32; width * height];
    let mut orientation = vec![0.0f32; width * height];

    for y in 1..height - 1 {
        for x in 1..width - 1 {
            let p00 = gray.data[(y - 1) * width + (x - 1)] as f32;
            let p01 = gray.data[(y - 1) * width + x] as f32;
            let p02 = gray.data[(y - 1) * width + (x + 1)] as f32;

            let p10 = gray.data[y * width + (x - 1)] as f32;
            let p12 = gray.data[y * width + (x + 1)] as f32;

            let p20 = gray.data[(y + 1) * width + (x - 1)] as f32;
            let p21 = gray.data[(y + 1) * width + x] as f32;
            let p22 = gray.data[(y + 1) * width + (x + 1)] as f32;

            // Scharr kernels:
            // Gx = [-3 0 3; -10 0 10; -3 0 3]
            // Gy = [-3 -10 -3; 0 0 0; 3 10 3]
            let val_gx =
                (3.0 * p02 + 10.0 * p12 + 3.0 * p22) - (3.0 * p00 + 10.0 * p10 + 3.0 * p20);
            let val_gy =
                (3.0 * p20 + 10.0 * p21 + 3.0 * p22) - (3.0 * p00 + 10.0 * p01 + 3.0 * p02);

            let idx = y * width + x;
            gx[idx] = val_gx;
            gy[idx] = val_gy;
            let mag = (val_gx * val_gx + val_gy * val_gy).sqrt();
            magnitude[idx] = mag;
            orientation[idx] = val_gy.atan2(val_gx);
        }
    }

    Ok(GradientMap { width: gray.width, height: gray.height, gx, gy, magnitude, orientation })
}

/// Canny Edge Detector with Gaussian smoothing, Sobel gradient, Non-Maximum Suppression, and Hysteresis.
pub fn canny_edge_detector(
    frame: &Frame,
    low_threshold: f32,
    high_threshold: f32,
) -> Result<Frame, VoxoraError> {
    let blurred = gaussian_blur(frame, 5, 1.4)?;
    let grad = sobel_operator(&blurred)?;

    let width = grad.width as usize;
    let height = grad.height as usize;

    // 1. Non-Maximum Suppression (NMS)
    let mut nms = vec![0.0f32; width * height];

    for y in 1..height - 1 {
        for x in 1..width - 1 {
            let idx = y * width + x;
            let mag = grad.magnitude[idx];
            if mag < low_threshold {
                continue;
            }

            // Quantize angle to 0, 45, 90, or 135 degrees
            let mut angle = grad.orientation[idx] * (180.0 / std::f32::consts::PI);
            if angle < 0.0 {
                angle += 180.0;
            }

            let (m1, m2) = if (0.0..22.5).contains(&angle) || (157.5..=180.0).contains(&angle) {
                (grad.magnitude[y * width + (x - 1)], grad.magnitude[y * width + (x + 1)])
            } else if (22.5..67.5).contains(&angle) {
                (
                    grad.magnitude[(y - 1) * width + (x + 1)],
                    grad.magnitude[(y + 1) * width + (x - 1)],
                )
            } else if (67.5..112.5).contains(&angle) {
                (grad.magnitude[(y - 1) * width + x], grad.magnitude[(y + 1) * width + x])
            } else {
                (
                    grad.magnitude[(y - 1) * width + (x - 1)],
                    grad.magnitude[(y + 1) * width + (x + 1)],
                )
            };

            if mag >= m1 && mag >= m2 {
                nms[idx] = mag;
            }
        }
    }

    // 2. Double Thresholding & Hysteresis
    let mut edge_data = vec![0u8; width * height];

    for y in 1..height - 1 {
        for x in 1..width - 1 {
            let idx = y * width + x;
            let val = nms[idx];

            if val >= high_threshold {
                edge_data[idx] = 255;
            } else if val >= low_threshold {
                // Check if connected to strong edge in 8-neighborhood
                let mut is_connected = false;
                for ny in (y - 1)..=(y + 1) {
                    for nx in (x - 1)..=(x + 1) {
                        if nms[ny * width + nx] >= high_threshold {
                            is_connected = true;
                            break;
                        }
                    }
                }
                if is_connected {
                    edge_data[idx] = 255;
                }
            }
        }
    }

    Frame::new(grad.width, grad.height, PixelFormat::Grayscale, edge_data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sobel_operator() {
        let mut data = vec![0u8; 100];
        // Vertical step edge at x=5
        for y in 0..10 {
            for x in 5..10 {
                data[y * 10 + x] = 255;
            }
        }
        let frame = Frame::new(10, 10, PixelFormat::Grayscale, data).unwrap();
        let grad = sobel_operator(&frame).unwrap();

        assert_eq!(grad.width, 10);
        // Magnitude at x=5 should be strong
        assert!(grad.magnitude[5 * 10 + 5] > 100.0);
    }

    #[test]
    fn test_canny_edge_detector() {
        let mut data = vec![0u8; 400];
        // Vertical line edge at x=10
        for y in 0..20 {
            for x in 10..20 {
                data[y * 20 + x] = 255;
            }
        }
        let frame = Frame::new(20, 20, PixelFormat::Grayscale, data).unwrap();
        let canny = canny_edge_detector(&frame, 20.0, 50.0).unwrap();

        assert_eq!(canny.width, 20);
        assert_eq!(canny.height, 20);
    }
}
