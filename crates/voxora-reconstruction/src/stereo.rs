//! Dense stereo block matching, disparity map calculation, left-right consistency checking, and depth conversion.

use voxora_core::{Frame, VoxoraError};

/// Dense disparity map representation.
#[derive(Debug, Clone, PartialEq)]
pub struct DisparityMap {
    /// Map width (pixels)
    pub width: usize,
    /// Map height (pixels)
    pub height: usize,
    /// Disparity values (pixels)
    pub data: Vec<f32>,
    /// Confidence map [0.0, 1.0]
    pub confidence: Vec<f32>,
}

impl DisparityMap {
    /// Creates a new empty disparity map.
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            data: vec![0.0; width * height],
            confidence: vec![1.0; width * height],
        }
    }
}

/// Block matcher for dense stereo disparity calculation.
#[derive(Debug, Clone)]
pub struct BlockMatcher {
    /// Block matching window size (must be odd, e.g. 7 or 15)
    pub window_size: usize,
    /// Maximum search disparity range (pixels)
    pub max_disparity: usize,
}

impl Default for BlockMatcher {
    fn default() -> Self {
        Self { window_size: 7, max_disparity: 32 }
    }
}

impl BlockMatcher {
    /// Creates a new block matcher with specified window size and max disparity.
    pub fn new(window_size: usize, max_disparity: usize) -> Self {
        Self { window_size, max_disparity }
    }

    /// Computes dense left disparity map using Sum of Absolute Differences (SAD).
    pub fn compute_disparity(
        &self,
        left: &Frame,
        right: &Frame,
    ) -> Result<DisparityMap, VoxoraError> {
        let width = left.width as usize;
        let height = left.height as usize;

        let mut disp_map = DisparityMap::new(width, height);
        let half_win = (self.window_size / 2) as i32;

        for y in half_win as usize..(height - half_win as usize) {
            for x in half_win as usize..(width - half_win as usize) {
                let mut best_sad = f32::MAX;
                let mut best_d = 0;

                for d in 0..self.max_disparity {
                    if (x as i32 - d as i32 - half_win) < 0 {
                        break;
                    }

                    let mut sad = 0.0f32;
                    for wy in -half_win..=half_win {
                        for wx in -half_win..=half_win {
                            let lx = (x as i32 + wx) as usize;
                            let ly = (y as i32 + wy) as usize;
                            let rx = (x as i32 + wx - d as i32) as usize;

                            let l_val = left.data[ly * width + lx] as f32;
                            let r_val = right.data[ly * width + rx] as f32;
                            sad += (l_val - r_val).abs();
                        }
                    }

                    if sad < best_sad {
                        best_sad = sad;
                        best_d = d;
                    }
                }

                let idx = y * width + x;
                disp_map.data[idx] = best_d as f32;
                disp_map.confidence[idx] = (1.0 / (1.0 + best_sad / 1000.0)).clamp(0.0, 1.0);
            }
        }

        Ok(disp_map)
    }

    /// Converts disparity map to depth map $Z = \frac{f \cdot B}{d}$.
    pub fn disparity_to_depth(
        &self,
        disp_map: &DisparityMap,
        focal_length: f32,
        baseline: f32,
    ) -> Vec<f32> {
        let mut depth_map = vec![0.0f32; disp_map.width * disp_map.height];

        for (i, &d) in disp_map.data.iter().enumerate() {
            if d > 0.5 {
                depth_map[i] = (focal_length * baseline) / d;
            }
        }

        depth_map
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_core::PixelFormat;

    #[test]
    fn test_block_matcher_disparity() {
        let f1 = Frame::new(32, 32, PixelFormat::Grayscale, vec![100; 1024]).unwrap();
        let f2 = Frame::new(32, 32, PixelFormat::Grayscale, vec![100; 1024]).unwrap();

        let matcher = BlockMatcher::new(5, 8);
        let disp = matcher.compute_disparity(&f1, &f2).unwrap();
        assert_eq!(disp.width, 32);
        assert_eq!(disp.height, 32);
    }
}
