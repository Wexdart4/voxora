//! Bilinear image warping, perspective spatial transformation, and disocclusion hole filling.

use voxora_core::{Frame, PixelFormat, VoxoraError};
use voxora_math::Matrix3x3;

/// Samples pixel color at sub-pixel coordinate $(u, v)$ using Bilinear Interpolation.
pub fn sample_bilinear(frame: &Frame, u: f64, v: f64) -> [u8; 3] {
    let width = frame.width as i32;
    let height = frame.height as i32;

    let x0 = u.floor() as i32;
    let y0 = v.floor() as i32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let dx = u - x0 as f64;
    let dy = v - y0 as f64;

    let get_pixel = |x: i32, y: i32| -> [f64; 3] {
        if x < 0 || x >= width || y < 0 || y >= height {
            return [0.0, 0.0, 0.0];
        }
        let idx = (y * width + x) as usize * 3;
        if idx + 2 < frame.data.len() {
            [frame.data[idx] as f64, frame.data[idx + 1] as f64, frame.data[idx + 2] as f64]
        } else {
            [0.0, 0.0, 0.0]
        }
    };

    let p00 = get_pixel(x0, y0);
    let p10 = get_pixel(x1, y0);
    let p01 = get_pixel(x0, y1);
    let p11 = get_pixel(x1, y1);

    let mut color = [0u8; 3];
    for c in 0..3 {
        let val = (1.0 - dx) * (1.0 - dy) * p00[c]
            + dx * (1.0 - dy) * p10[c]
            + (1.0 - dx) * dy * p01[c]
            + dx * dy * p11[c];
        color[c] = val.clamp(0.0, 255.0) as u8;
    }

    color
}

/// Warps an input video frame using perspective transformation matrix $H$.
pub fn warp_perspective(frame: &Frame, homography: &Matrix3x3) -> Result<Frame, VoxoraError> {
    let width = frame.width as usize;
    let height = frame.height as usize;
    let mut out_data = vec![0u8; width * height * 3];

    // Compute inverse homography matrix
    let h_inv = homography.invert().ok_or_else(|| {
        VoxoraError::GeometryError("Homography matrix singular and non-invertible".into())
    })?;

    for y in 0..height {
        for x in 0..width {
            let p_out = voxora_math::Vector3::new(x as f64, y as f64, 1.0);
            let p_in_hom = h_inv.mul_vec(p_out);

            let src_u = p_in_hom.x / p_in_hom.z.max(1e-6);
            let src_v = p_in_hom.y / p_in_hom.z.max(1e-6);

            let color = sample_bilinear(frame, src_u, src_v);
            let out_idx = (y * width + x) * 3;
            out_data[out_idx] = color[0];
            out_data[out_idx + 1] = color[1];
            out_data[out_idx + 2] = color[2];
        }
    }

    Frame::new(frame.width, frame.height, PixelFormat::Rgb8, out_data)
}

/// Fills unprojected disocclusion holes (zero pixels) using 3x3 median/average neighborhood spatial interpolation.
pub fn fill_projection_holes(frame: &Frame) -> Result<Frame, VoxoraError> {
    let width = frame.width as usize;
    let height = frame.height as usize;
    let mut out_data = frame.data.clone();

    for y in 1..(height - 1) {
        for x in 1..(width - 1) {
            let idx = (y * width + x) * 3;
            if out_data[idx] == 0 && out_data[idx + 1] == 0 && out_data[idx + 2] == 0 {
                let mut sum_r = 0u32;
                let mut sum_g = 0u32;
                let mut sum_b = 0u32;
                let mut valid_count = 0u32;

                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let n_idx =
                            ((y as i32 + dy) as usize * width + (x as i32 + dx) as usize) * 3;
                        if out_data[n_idx] > 0 || out_data[n_idx + 1] > 0 || out_data[n_idx + 2] > 0
                        {
                            sum_r += out_data[n_idx] as u32;
                            sum_g += out_data[n_idx + 1] as u32;
                            sum_b += out_data[n_idx + 2] as u32;
                            valid_count += 1;
                        }
                    }
                }

                if let Some(nz_count) = std::num::NonZeroU32::new(valid_count) {
                    out_data[idx] = (sum_r / nz_count) as u8;
                    out_data[idx + 1] = (sum_g / nz_count) as u8;
                    out_data[idx + 2] = (sum_b / nz_count) as u8;
                }
            }
        }
    }

    Frame::new(frame.width, frame.height, frame.format, out_data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_warp_perspective() {
        let f = Frame::new(16, 16, PixelFormat::Rgb8, vec![100; 16 * 16 * 3]).unwrap();
        let h = Matrix3x3::IDENTITY;
        let warped = warp_perspective(&f, &h).unwrap();
        assert_eq!(warped.width, 16);
    }
}
