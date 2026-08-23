//! Pyramidal Lucas-Kanade optical flow, subpixel refinement, forward-backward validation, and feature tracking.

use crate::edge::sobel_operator;
use crate::filter::GaussianPyramid;
use crate::FeaturePoint;
use voxora_core::{Frame, VoxoraError};

/// Motion vector estimated by optical flow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowVector {
    /// Horizontal displacement delta_x
    pub dx: f32,
    /// Vertical displacement delta_y
    pub dy: f32,
    /// Flow estimation confidence score [0.0, 1.0]
    pub confidence: f32,
    /// Whether feature was successfully tracked
    pub is_tracked: bool,
}

/// Multi-frame trajectory track of a feature point across video sequence.
#[derive(Debug, Clone, PartialEq)]
pub struct FeatureTrack {
    /// Unique track identifier
    pub track_id: usize,
    /// Trajectory points as (frame_id, FeaturePoint) pairs
    pub trajectory: Vec<(usize, FeaturePoint)>,
    /// Track age in frames
    pub age: usize,
    /// Is track active or terminated
    pub is_active: bool,
}

impl FeatureTrack {
    /// Creates a new feature track.
    pub fn new(track_id: usize, frame_id: usize, point: FeaturePoint) -> Self {
        Self { track_id, trajectory: vec![(frame_id, point)], age: 1, is_active: true }
    }

    /// Appends a newly tracked point to trajectory.
    pub fn add_point(&mut self, frame_id: usize, point: FeaturePoint) {
        self.trajectory.push((frame_id, point));
        self.age += 1;
    }
}

/// Solves Pyramidal Lucas-Kanade optical flow for a list of feature points across two frames.
pub fn pyramidal_lucas_kanade(
    prev_frame: &Frame,
    curr_frame: &Frame,
    points: &[FeaturePoint],
    pyramid_levels: usize,
    window_size: usize,
    max_iterations: usize,
) -> Result<Vec<FlowVector>, VoxoraError> {
    let prev_pyramid = GaussianPyramid::build(prev_frame, pyramid_levels)?;
    let curr_pyramid = GaussianPyramid::build(curr_frame, pyramid_levels)?;

    let num_levels = prev_pyramid.levels.len().min(curr_pyramid.levels.len());
    let mut prev_grads = Vec::with_capacity(num_levels);
    for lvl in &prev_pyramid.levels {
        prev_grads.push(sobel_operator(lvl)?);
    }

    let mut flow_vectors = Vec::with_capacity(points.len());
    let half_win = (window_size / 2) as i32;

    for pt in points {
        let mut g_x = pt.x;
        let mut g_y = pt.y;

        let mut tracked = true;
        let mut final_conf = 1.0f32;

        // Coarse-to-fine pyramid tracking
        for level in (0..num_levels).rev() {
            let scale = 1.0 / (1 << level) as f32;
            let p_x = g_x * scale;
            let p_y = g_y * scale;

            let prev_lvl = &prev_pyramid.levels[level];
            let curr_lvl = &curr_pyramid.levels[level];
            let grad = &prev_grads[level];

            let w = prev_lvl.width as i32;
            let h = prev_lvl.height as i32;

            let ix = p_x.round() as i32;
            let iy = p_y.round() as i32;

            if ix - half_win < 0 || ix + half_win >= w || iy - half_win < 0 || iy + half_win >= h {
                tracked = false;
                break;
            }

            // Compute spatial gradient tensor G = [sum(Ix^2) sum(Ix*Iy); sum(Ix*Iy) sum(Iy^2)]
            let mut ixx = 0.0f32;
            let mut iyy = 0.0f32;
            let mut ixy = 0.0f32;

            for wy in -half_win..=half_win {
                for wx in -half_win..=half_win {
                    let idx = ((iy + wy) * w + (ix + wx)) as usize;
                    let gx_val = grad.gx[idx];
                    let gy_val = grad.gy[idx];
                    ixx += gx_val * gx_val;
                    iyy += gy_val * gy_val;
                    ixy += gx_val * gy_val;
                }
            }

            let det = ixx * iyy - ixy * ixy;
            if det.abs() < 1e-4 {
                tracked = false;
                break;
            }

            let inv_det = 1.0 / det;
            let mut u = 0.0f32;
            let mut v = 0.0f32;

            // Iterative LK flow refinement
            for _iter in 0..max_iterations {
                let mut b1 = 0.0f32;
                let mut b2 = 0.0f32;

                for wy in -half_win..=half_win {
                    for wx in -half_win..=half_win {
                        let prev_idx = ((iy + wy) * w + (ix + wx)) as usize;
                        let curr_x =
                            (ix as f32 + wx as f32 + u).round().clamp(0.0, (w - 1) as f32) as usize;
                        let curr_y =
                            (iy as f32 + wy as f32 + v).round().clamp(0.0, (h - 1) as f32) as usize;
                        let curr_idx = curr_y * (w as usize) + curr_x;

                        let it = curr_lvl.data[curr_idx] as f32 - prev_lvl.data[prev_idx] as f32;
                        let gx_val = grad.gx[prev_idx];
                        let gy_val = grad.gy[prev_idx];

                        b1 += -gx_val * it;
                        b2 += -gy_val * it;
                    }
                }

                // du = inv(G) * b
                let du = inv_det * (iyy * b1 - ixy * b2);
                let dv = inv_det * (-ixy * b1 + ixx * b2);

                u += du;
                v += dv;

                if du * du + dv * dv < 1e-3 {
                    break;
                }
            }

            g_x += u * (1 << level) as f32;
            g_y += v * (1 << level) as f32;
            final_conf = (det / (ixx + iyy + 1e-5)).clamp(0.0, 1.0);
        }

        flow_vectors.push(FlowVector {
            dx: g_x - pt.x,
            dy: g_y - pt.y,
            confidence: final_conf,
            is_tracked: tracked,
        });
    }

    Ok(flow_vectors)
}

/// Performs forward-backward optical flow check to reject inaccurate or occluded feature tracks.
pub fn forward_backward_flow_check(
    prev_frame: &Frame,
    curr_frame: &Frame,
    points: &[FeaturePoint],
    max_drift_distance: f32,
) -> Result<Vec<bool>, VoxoraError> {
    let fwd_flow = pyramidal_lucas_kanade(prev_frame, curr_frame, points, 3, 7, 10)?;

    let mut tracked_curr_pts = Vec::new();
    let mut tracked_indices = Vec::new();

    for (idx, (pt, flow)) in points.iter().zip(fwd_flow.iter()).enumerate() {
        if flow.is_tracked {
            let mut next_pt = pt.clone();
            next_pt.x += flow.dx;
            next_pt.y += flow.dy;
            tracked_curr_pts.push(next_pt);
            tracked_indices.push(idx);
        }
    }

    if tracked_curr_pts.is_empty() {
        return Ok(vec![false; points.len()]);
    }

    let bwd_flow = pyramidal_lucas_kanade(curr_frame, prev_frame, &tracked_curr_pts, 3, 7, 10)?;

    let mut valid_mask = vec![false; points.len()];

    for (t_idx, bwd) in bwd_flow.iter().enumerate() {
        let orig_idx = tracked_indices[t_idx];
        if bwd.is_tracked {
            let fwd = fwd_flow[orig_idx];
            let loop_dx = fwd.dx + bwd.dx;
            let loop_dy = fwd.dy + bwd.dy;
            let drift = (loop_dx * loop_dx + loop_dy * loop_dy).sqrt();

            if drift <= max_drift_distance {
                valid_mask[orig_idx] = true;
            }
        }
    }

    Ok(valid_mask)
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_core::PixelFormat;

    #[test]
    fn test_pyramidal_lucas_kanade() {
        let f1 = Frame::new(32, 32, PixelFormat::Grayscale, vec![100; 1024]).unwrap();
        let f2 = Frame::new(32, 32, PixelFormat::Grayscale, vec![100; 1024]).unwrap();

        let pt = FeaturePoint::new(16.0, 16.0, 100.0);
        let flow = pyramidal_lucas_kanade(&f1, &f2, &[pt], 2, 5, 5).unwrap();

        assert_eq!(flow.len(), 1);
    }
}
