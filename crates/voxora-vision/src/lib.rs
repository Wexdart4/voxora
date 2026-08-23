//! Computer vision algorithms, filtering, edge detection, corner detection, feature matching, RANSAC, and optical flow.

#![warn(missing_docs)]

pub mod descriptor;
pub mod edge;
pub mod filter;
pub mod flow;
pub mod matching;
pub mod ransac;
pub mod segmentation;
pub mod temporal;

pub use descriptor::{compute_binary_descriptor, Descriptor};
pub use edge::sobel_operator;
pub use segmentation::{CutType, DynamicMaskEstimator, SceneCutDetector};
pub use temporal::TemporalStabilizer;
use voxora_core::Frame;

/// Represents a detected 2D feature point in image space with descriptor and tracking metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct FeaturePoint {
    /// Pixel column (X coordinate)
    pub x: f32,
    /// Pixel row (Y coordinate)
    pub y: f32,
    /// Octave scale level
    pub scale: f32,
    /// Orientation angle in radians
    pub orientation: f32,
    /// Feature response or corner strength
    pub response: f32,
    /// Frame sequence identifier
    pub frame_id: usize,
    /// Associated feature descriptor
    pub descriptor: Option<Descriptor>,
}

impl FeaturePoint {
    /// Creates a new feature point.
    pub fn new(x: f32, y: f32, response: f32) -> Self {
        Self { x, y, scale: 1.0, orientation: 0.0, response, frame_id: 0, descriptor: None }
    }
}

/// Abstract feature detector trait.
pub trait FeatureDetector {
    /// Detects feature points within a frame.
    fn detect(&self, frame: &Frame) -> Vec<FeaturePoint>;
}

/// Harris Corner Detector based on autocorrelation structure tensor M.
#[derive(Debug, Clone)]
pub struct HarrisCornerDetector {
    /// Response threshold for corner detection
    pub threshold: f32,
    /// Empirical sensitivity constant k (typically 0.04 - 0.06)
    pub k: f32,
    /// Maximum feature count limit
    pub max_features: usize,
}

impl Default for HarrisCornerDetector {
    fn default() -> Self {
        Self { threshold: 10000.0, k: 0.04, max_features: 500 }
    }
}

impl FeatureDetector for HarrisCornerDetector {
    fn detect(&self, frame: &Frame) -> Vec<FeaturePoint> {
        let grad = match sobel_operator(frame) {
            Ok(g) => g,
            Err(_) => return Vec::new(),
        };

        let width = grad.width as usize;
        let height = grad.height as usize;

        let mut corners = Vec::new();

        for y in 2..height - 2 {
            for x in 2..width - 2 {
                let mut ixx = 0.0f32;
                let mut iyy = 0.0f32;
                let mut ixy = 0.0f32;

                for wy in -1..=1 {
                    for wx in -1..=1 {
                        let idx = ((y as i32 + wy) * width as i32 + (x as i32 + wx)) as usize;
                        let gx = grad.gx[idx];
                        let gy = grad.gy[idx];
                        ixx += gx * gx;
                        iyy += gy * gy;
                        ixy += gx * gy;
                    }
                }

                let det = ixx * iyy - ixy * ixy;
                let trace = ixx + iyy;
                let response = det - self.k * trace * trace;

                if response > self.threshold {
                    let mut pt = FeaturePoint::new(x as f32, y as f32, response);
                    pt.descriptor = compute_binary_descriptor(frame, &pt).map(Descriptor::Binary);
                    corners.push(pt);
                }
            }
        }

        corners.sort_by(|a, b| {
            b.response.partial_cmp(&a.response).unwrap_or(std::cmp::Ordering::Equal)
        });
        corners.truncate(self.max_features);
        corners
    }
}

/// Shi-Tomasi Corner Detector based on minimum eigenvalue min(lambda1, lambda2).
#[derive(Debug, Clone)]
pub struct ShiTomasiDetector {
    /// Minimum eigenvalue threshold
    pub threshold: f32,
    /// Maximum feature count limit
    pub max_features: usize,
}

impl Default for ShiTomasiDetector {
    fn default() -> Self {
        Self { threshold: 500.0, max_features: 500 }
    }
}

impl FeatureDetector for ShiTomasiDetector {
    fn detect(&self, frame: &Frame) -> Vec<FeaturePoint> {
        let grad = match sobel_operator(frame) {
            Ok(g) => g,
            Err(_) => return Vec::new(),
        };

        let width = grad.width as usize;
        let height = grad.height as usize;

        let mut corners = Vec::new();

        for y in 2..height - 2 {
            for x in 2..width - 2 {
                let mut ixx = 0.0f32;
                let mut iyy = 0.0f32;
                let mut ixy = 0.0f32;

                for wy in -1..=1 {
                    for wx in -1..=1 {
                        let idx = ((y as i32 + wy) * width as i32 + (x as i32 + wx)) as usize;
                        let gx = grad.gx[idx];
                        let gy = grad.gy[idx];
                        ixx += gx * gx;
                        iyy += gy * gy;
                        ixy += gx * gy;
                    }
                }

                let trace = ixx + iyy;
                let det = ixx * iyy - ixy * ixy;
                let disc = (trace * trace - 4.0 * det).max(0.0).sqrt();
                let min_eigenvalue = (trace - disc) / 2.0;

                if min_eigenvalue > self.threshold {
                    let mut pt = FeaturePoint::new(x as f32, y as f32, min_eigenvalue);
                    pt.descriptor = compute_binary_descriptor(frame, &pt).map(Descriptor::Binary);
                    corners.push(pt);
                }
            }
        }

        corners.sort_by(|a, b| {
            b.response.partial_cmp(&a.response).unwrap_or(std::cmp::Ordering::Equal)
        });
        corners.truncate(self.max_features);
        corners
    }
}

/// FAST Corner Detector.
#[derive(Debug, Clone)]
pub struct FastFeatureDetector {
    /// Intensity threshold
    pub threshold: u8,
    /// Contiguous circle pixels
    pub contiguous_pixels: usize,
}

impl Default for FastFeatureDetector {
    fn default() -> Self {
        Self { threshold: 20, contiguous_pixels: 9 }
    }
}

impl FeatureDetector for FastFeatureDetector {
    fn detect(&self, frame: &Frame) -> Vec<FeaturePoint> {
        let gray = frame.to_grayscale();
        let width = gray.width as i32;
        let height = gray.height as i32;

        let circle_offsets: [(i32, i32); 16] = [
            (0, -3),
            (1, -3),
            (2, -2),
            (3, -1),
            (3, 0),
            (3, 1),
            (2, 2),
            (1, 3),
            (0, 3),
            (-1, 3),
            (-2, 2),
            (-3, 1),
            (-3, 0),
            (-3, -1),
            (-2, -2),
            (-1, -3),
        ];

        let mut corners = Vec::new();

        for y in 3..height - 3 {
            for x in 3..width - 3 {
                let center_val = gray.data[(y * width + x) as usize] as i16;
                let thresh = self.threshold as i16;

                let p0 = gray.data
                    [((y + circle_offsets[0].1) * width + (x + circle_offsets[0].0)) as usize]
                    as i16;
                let p4 = gray.data
                    [((y + circle_offsets[4].1) * width + (x + circle_offsets[4].0)) as usize]
                    as i16;
                let p8 = gray.data
                    [((y + circle_offsets[8].1) * width + (x + circle_offsets[8].0)) as usize]
                    as i16;
                let p12 = gray.data
                    [((y + circle_offsets[12].1) * width + (x + circle_offsets[12].0)) as usize]
                    as i16;

                let count_bright = ((p0 > center_val + thresh) as usize)
                    + ((p4 > center_val + thresh) as usize)
                    + ((p8 > center_val + thresh) as usize)
                    + ((p12 > center_val + thresh) as usize);

                let count_dark = ((p0 < center_val - thresh) as usize)
                    + ((p4 < center_val - thresh) as usize)
                    + ((p8 < center_val - thresh) as usize)
                    + ((p12 < center_val - thresh) as usize);

                if count_bright < 3 && count_dark < 3 {
                    continue;
                }

                let mut ring_vals = [0i16; 16];
                for i in 0..16 {
                    ring_vals[i] = gray.data
                        [((y + circle_offsets[i].1) * width + (x + circle_offsets[i].0)) as usize]
                        as i16;
                }

                let mut is_corner = false;

                for start in 0..16 {
                    let mut contiguous = true;
                    for k in 0..self.contiguous_pixels {
                        let idx = (start + k) % 16;
                        if ring_vals[idx] <= center_val + thresh {
                            contiguous = false;
                            break;
                        }
                    }
                    if contiguous {
                        is_corner = true;
                        break;
                    }
                }

                if is_corner {
                    let response = (center_val - p0).abs() as f32;
                    let mut pt = FeaturePoint::new(x as f32, y as f32, response);
                    pt.descriptor = compute_binary_descriptor(frame, &pt).map(Descriptor::Binary);
                    corners.push(pt);
                }
            }
        }

        corners
    }
}

/// Spatial Grid Filter.
#[derive(Debug, Clone)]
pub struct SpatialGridFilter {
    /// Number of grid columns
    pub grid_cols: usize,
    /// Number of grid rows
    pub grid_rows: usize,
    /// Max features per grid cell
    pub max_per_cell: usize,
}

impl SpatialGridFilter {
    /// Creates a spatial grid filter.
    pub fn new(grid_cols: usize, grid_rows: usize, max_per_cell: usize) -> Self {
        Self { grid_cols, grid_rows, max_per_cell }
    }

    /// Filters feature points evenly across spatial image grid cells.
    pub fn filter(&self, points: &[FeaturePoint], width: u32, height: u32) -> Vec<FeaturePoint> {
        if points.is_empty() || width == 0 || height == 0 {
            return Vec::new();
        }

        let cell_w = (width as f32 / self.grid_cols as f32).max(1.0);
        let cell_h = (height as f32 / self.grid_rows as f32).max(1.0);

        let mut grid = vec![Vec::<FeaturePoint>::new(); self.grid_cols * self.grid_rows];

        for pt in points {
            let col = (pt.x / cell_w).floor() as usize;
            let row = (pt.y / cell_h).floor() as usize;
            let c_idx = col.min(self.grid_cols - 1);
            let r_idx = row.min(self.grid_rows - 1);
            let cell_idx = r_idx * self.grid_cols + c_idx;
            grid[cell_idx].push(pt.clone());
        }

        let mut filtered = Vec::new();

        for cell in &mut grid {
            cell.sort_by(|a, b| {
                b.response.partial_cmp(&a.response).unwrap_or(std::cmp::Ordering::Equal)
            });
            for pt in cell.iter().take(self.max_per_cell) {
                filtered.push(pt.clone());
            }
        }

        filtered
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_core::PixelFormat;

    #[test]
    fn test_harris_corner_detector() {
        let mut data = vec![0u8; 100 * 100];
        for y in 30..70 {
            for x in 30..70 {
                data[y * 100 + x] = 255;
            }
        }
        let frame = Frame::new(100, 100, PixelFormat::Grayscale, data).unwrap();

        let detector = HarrisCornerDetector { threshold: 1000.0, k: 0.04, max_features: 50 };

        let corners = detector.detect(&frame);
        assert!(!corners.is_empty());
    }
}
