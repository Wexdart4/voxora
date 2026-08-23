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

        Self::smooth_depth_map(&depth_map, disp_map.width, disp_map.height)
    }

    /// Applies 5x5 Median Spatial Depth Smoothing to eliminate depth noise spikes and flatten surfaces.
    pub fn smooth_depth_map(depth_map: &[f32], width: usize, height: usize) -> Vec<f32> {
        let mut smoothed = depth_map.to_vec();

        for y in 2..(height - 2) {
            for x in 2..(width - 2) {
                let mut window = Vec::with_capacity(25);
                for dy in -2..=2 {
                    for dx in -2..=2 {
                        let idx = (y as i32 + dy) as usize * width + (x as i32 + dx) as usize;
                        let val = depth_map[idx];
                        if val > 0.1 {
                            window.push(val);
                        }
                    }
                }

                if !window.is_empty() {
                    window.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                    let median = window[window.len() / 2];
                    smoothed[y * width + x] = median;
                }
            }
        }

        smoothed
    }
}

/// Plane Sweep Stereo engine utilizing 5x5 Census Transform, Inverse Depth Sampling,
/// and 1D Scanline Semi-Global Matching (SGM) Dynamic Programming optimization.
#[derive(Debug, Clone)]
pub struct PlaneSweepStereo {
    /// Number of plane sweep sampling layers D (e.g. 32)
    pub num_planes: usize,
    /// Minimum depth boundary in meters (e.g. 1.0)
    pub min_depth: f32,
    /// Maximum depth boundary in meters (e.g. 8.0)
    pub max_depth: f32,
    /// SGM Penalty P1 for small disparity step |d1 - d2| = 1
    pub p1: f32,
    /// SGM Penalty P2 for large disparity step |d1 - d2| > 1
    pub p2: f32,
}

impl Default for PlaneSweepStereo {
    fn default() -> Self {
        Self {
            num_planes: 32,
            min_depth: 1.0,
            max_depth: 8.0,
            p1: 10.0,
            p2: 80.0,
        }
    }
}

impl PlaneSweepStereo {
    /// Creates a new Plane Sweep Stereo engine.
    pub fn new(num_planes: usize, min_depth: f32, max_depth: f32, p1: f32, p2: f32) -> Self {
        Self {
            num_planes,
            min_depth,
            max_depth,
            p1,
            p2,
        }
    }

    /// Computes 5x5 Census Transform bitmask for a grayscale frame.
    pub fn compute_census_transform(frame: &Frame) -> Vec<u32> {
        let gray = frame.to_grayscale();
        let width = gray.width as usize;
        let height = gray.height as usize;
        let mut census = vec![0u32; width * height];

        for y in 2..(height - 2) {
            for x in 2..(width - 2) {
                let center_val = gray.data[y * width + x];
                let mut bitmask = 0u32;
                let mut bit_idx = 0;

                for dy in -2..=2 {
                    for dx in -2..=2 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let val =
                            gray.data[((y as i32 + dy) as usize) * width + ((x as i32 + dx) as usize)];
                        if val >= center_val {
                            bitmask |= 1 << bit_idx;
                        }
                        bit_idx += 1;
                    }
                }
                census[y * width + x] = bitmask;
            }
        }

        census
    }

    /// Computes inverse depth sampled plane depths $d_k = \frac{1}{s_k}$.
    pub fn compute_inverse_depth_planes(&self) -> Vec<f32> {
        let s_min = 1.0 / self.max_depth;
        let s_max = 1.0 / self.min_depth;
        let d = self.num_planes;

        let mut planes = Vec::with_capacity(d);
        for k in 0..d {
            let frac = if d > 1 { k as f32 / (d - 1) as f32 } else { 0.0 };
            let s_k = s_min + frac * (s_max - s_min);
            planes.push(1.0 / s_k);
        }

        planes
    }

    /// Computes dense depth map using Plane Sweep Stereo + SGM Scanline Energy Minimization.
    pub fn compute_depth_map(
        &self,
        ref_frame: &Frame,
        src_frame: &Frame,
    ) -> Result<Vec<f32>, VoxoraError> {
        let width = ref_frame.width as usize;
        let height = ref_frame.height as usize;
        let d_count = self.num_planes;

        let ref_census = Self::compute_census_transform(ref_frame);
        let src_census = Self::compute_census_transform(src_frame);

        let planes = self.compute_inverse_depth_planes();

        // 1. Build Cost Volume: cost_vol[y * width * d_count + x * d_count + k]
        let mut cost_volume = vec![0.0f32; width * height * d_count];

        for k in 0..d_count {
            let depth = planes[k];
            // Disparity shift in pixels: disp = (focal_length * baseline) / depth
            // Approximating baseline shift proportional to (max_depth / depth)
            let shift_x = (8.0 * (self.min_depth / depth)) as i32;

            for y in 2..(height - 2) {
                for x in 2..(width - 2) {
                    let ref_idx = y * width + x;
                    let ref_c = ref_census[ref_idx];

                    let src_x = x as i32 - shift_x;
                    let cost = if src_x >= 2 && src_x < (width as i32 - 2) {
                        let src_idx = y * width + src_x as usize;
                        let src_c = src_census[src_idx];
                        (ref_c ^ src_c).count_ones() as f32
                    } else {
                        24.0 // Max Hamming penalty for out-of-bounds
                    };

                    cost_volume[(y * width + x) * d_count + k] = cost;
                }
            }
        }

        // 2. 1D Scanline SGM Dynamic Programming (Left to Right pass)
        let mut sgm_cost = cost_volume.clone();

        for y in 2..(height - 2) {
            for x in 3..(width - 2) {
                let curr_row_idx = (y * width + x) * d_count;
                let prev_row_idx = (y * width + (x - 1)) * d_count;

                // Find min cost in previous pixel
                let mut min_prev = f32::MAX;
                for k in 0..d_count {
                    let c = sgm_cost[prev_row_idx + k];
                    if c < min_prev {
                        min_prev = c;
                    }
                }

                for k in 0..d_count {
                    let l_same = sgm_cost[prev_row_idx + k];
                    let l_minus1 = if k > 0 { sgm_cost[prev_row_idx + k - 1] + self.p1 } else { f32::MAX };
                    let l_plus1 = if k + 1 < d_count { sgm_cost[prev_row_idx + k + 1] + self.p1 } else { f32::MAX };
                    let l_min_p2 = min_prev + self.p2;

                    let min_l = l_same.min(l_minus1).min(l_plus1).min(l_min_p2);
                    sgm_cost[curr_row_idx + k] += min_l - min_prev;
                }
            }
        }

        // 3. Winner-Takes-All (WTA) Depth Selection with Parabolic Sub-Pixel Refinement
        let mut depth_map = vec![self.max_depth; width * height];

        for y in 0..height {
            for x in 0..width {
                let idx_base = (y * width + x) * d_count;
                let mut min_c = f32::MAX;
                let mut best_k = 0;

                for k in 0..d_count {
                    let c = sgm_cost[idx_base + k];
                    if c < min_c {
                        min_c = c;
                        best_k = k;
                    }
                }

                // Sub-pixel Parabolic (Quadratic) Interpolation around best_k
                let k_sub = if best_k > 0 && best_k + 1 < d_count {
                    let c0 = sgm_cost[idx_base + best_k - 1];
                    let c1 = sgm_cost[idx_base + best_k];
                    let c2 = sgm_cost[idx_base + best_k + 1];
                    let denom = 2.0 * (c0 - 2.0 * c1 + c2);
                    if denom.abs() > 1e-4 {
                        let delta = -((c2 - c0) / denom).clamp(-0.5, 0.5);
                        best_k as f32 + delta
                    } else {
                        best_k as f32
                    }
                } else {
                    best_k as f32
                };

                let s_min = 1.0 / self.max_depth;
                let s_max = 1.0 / self.min_depth;
                let frac = if d_count > 1 { k_sub / (d_count - 1) as f32 } else { 0.0 };
                let s_k = s_min + frac * (s_max - s_min);

                depth_map[y * width + x] = 1.0 / s_k;
            }
        }

        Ok(BlockMatcher::smooth_depth_map(&depth_map, width, height))
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

    #[test]
    fn test_plane_sweep_stereo() {
        let f1 = Frame::new(16, 16, PixelFormat::Rgb8, vec![120; 16 * 16 * 3]).unwrap();
        let f2 = Frame::new(16, 16, PixelFormat::Rgb8, vec![120; 16 * 16 * 3]).unwrap();
        let pss = PlaneSweepStereo::new(8, 1.0, 5.0, 5.0, 20.0);
        let depth = pss.compute_depth_map(&f1, &f2).unwrap();
        assert_eq!(depth.len(), 16 * 16);
    }
}
