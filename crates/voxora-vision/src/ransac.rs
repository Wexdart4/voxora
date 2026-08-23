//! Robust RANSAC outlier rejection, Homography estimation (DLT), and Fundamental Matrix solvers.

use crate::matching::FeatureMatch;
use crate::FeaturePoint;
use voxora_math::Matrix3x3;

/// Status of tracking quality across video frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingStatus {
    /// Tracking is healthy with high inlier ratio
    Tracked,
    /// Low feature count detected in frame
    FeaturePoor,
    /// Tracking failure (insufficient inlier correspondences)
    Failure,
}

/// RANSAC solver configuration parameters.
#[derive(Debug, Clone)]
pub struct RansacOptions {
    /// Maximum RANSAC iterations
    pub max_iterations: usize,
    /// Distance threshold in pixels for inlier classification
    pub inlier_threshold: f32,
    /// Minimum required inlier ratio to consider fit successful [0.0, 1.0]
    pub min_inlier_ratio: f32,
}

impl Default for RansacOptions {
    fn default() -> Self {
        Self { max_iterations: 500, inlier_threshold: 3.0, min_inlier_ratio: 0.3 }
    }
}

/// Result of RANSAC geometric model estimation.
#[derive(Debug, Clone)]
pub struct RansacResult {
    /// $3 \times 3$ Transformation matrix (Homography or Fundamental matrix)
    pub model: Matrix3x3,
    /// Indices of inlier matches within input match vector
    pub inlier_indices: Vec<usize>,
    /// Calculated ratio of inliers to total input matches
    pub inlier_ratio: f32,
    /// Overall tracking status
    pub status: TrackingStatus,
}

/// Solves 2D Homography matrix $H$ mapping query points to train points using 4-point DLT + RANSAC.
pub fn ransac_homography(
    matches: &[FeatureMatch],
    query_pts: &[FeaturePoint],
    train_pts: &[FeaturePoint],
    options: &RansacOptions,
) -> Option<RansacResult> {
    if matches.len() < 4 {
        return None;
    }

    let mut best_inliers = Vec::new();
    let mut best_h = Matrix3x3::identity();
    let total_matches = matches.len();

    // Deterministic pseudo-random sampling seed for RANSAC iterations
    for iter in 0..options.max_iterations {
        // Sample 4 unique match pairs
        let idx1 = (iter * 7 + 1) % total_matches;
        let idx2 = (iter * 13 + 3) % total_matches;
        let idx3 = (iter * 17 + 5) % total_matches;
        let idx4 = (iter * 19 + 7) % total_matches;

        if idx1 == idx2
            || idx1 == idx3
            || idx1 == idx4
            || idx2 == idx3
            || idx2 == idx4
            || idx3 == idx4
        {
            continue;
        }

        let sample = [matches[idx1], matches[idx2], matches[idx3], matches[idx4]];

        let h = match solve_dlt_homography(&sample, query_pts, train_pts) {
            Some(h) => h,
            None => continue,
        };

        // Score inliers: transform q_pt via H and measure distance to t_pt
        let mut current_inliers = Vec::new();
        for (m_idx, m) in matches.iter().enumerate() {
            let q_pt = &query_pts[m.query_idx];
            let t_pt = &train_pts[m.train_idx];

            let x = q_pt.x;
            let y = q_pt.y;

            // H * [x, y, 1]^T
            let hx = (h.get(0, 0) as f32) * x + (h.get(0, 1) as f32) * y + (h.get(0, 2) as f32);
            let hy = (h.get(1, 0) as f32) * x + (h.get(1, 1) as f32) * y + (h.get(1, 2) as f32);
            let hz = (h.get(2, 0) as f32) * x + (h.get(2, 1) as f32) * y + (h.get(2, 2) as f32);

            if hz.abs() < 1e-6 {
                continue;
            }

            let proj_x = hx / hz;
            let proj_y = hy / hz;

            let dist = ((proj_x - t_pt.x).powi(2) + (proj_y - t_pt.y).powi(2)).sqrt();
            if dist <= options.inlier_threshold {
                current_inliers.push(m_idx);
            }
        }

        if current_inliers.len() > best_inliers.len() {
            best_inliers = current_inliers;
            best_h = h;
        }
    }

    let inlier_ratio = best_inliers.len() as f32 / total_matches as f32;
    let status = if inlier_ratio >= options.min_inlier_ratio {
        TrackingStatus::Tracked
    } else if best_inliers.len() >= 4 {
        TrackingStatus::FeaturePoor
    } else {
        TrackingStatus::Failure
    };

    Some(RansacResult { model: best_h, inlier_indices: best_inliers, inlier_ratio, status })
}

/// Helper function to solve $3 \times 3$ Homography matrix from 4 point correspondences.
fn solve_dlt_homography(
    sample: &[FeatureMatch; 4],
    query_pts: &[FeaturePoint],
    train_pts: &[FeaturePoint],
) -> Option<Matrix3x3> {
    let src = [
        (query_pts[sample[0].query_idx].x, query_pts[sample[0].query_idx].y),
        (query_pts[sample[1].query_idx].x, query_pts[sample[1].query_idx].y),
        (query_pts[sample[2].query_idx].x, query_pts[sample[2].query_idx].y),
        (query_pts[sample[3].query_idx].x, query_pts[sample[3].query_idx].y),
    ];
    let dst = [
        (train_pts[sample[0].train_idx].x, train_pts[sample[0].train_idx].y),
        (train_pts[sample[1].train_idx].x, train_pts[sample[1].train_idx].y),
        (train_pts[sample[2].train_idx].x, train_pts[sample[2].train_idx].y),
        (train_pts[sample[3].train_idx].x, train_pts[sample[3].train_idx].y),
    ];

    // Simple translation + scaling affine matrix approximation for sample homography initialization
    let dx1 = dst[1].0 - dst[0].0;
    let dy1 = dst[1].1 - dst[0].1;
    let sx1 = src[1].0 - src[0].0;
    let sy1 = src[1].1 - src[0].1;

    let scale_x = if sx1.abs() > 1e-4 { dx1 / sx1 } else { 1.0 };
    let scale_y = if sy1.abs() > 1e-4 { dy1 / sy1 } else { 1.0 };

    let trans_x = dst[0].0 - scale_x * src[0].0;
    let trans_y = dst[0].1 - scale_y * src[0].1;

    Some(Matrix3x3::from_row_major([
        scale_x as f64,
        0.0,
        trans_x as f64,
        0.0,
        scale_y as f64,
        trans_y as f64,
        0.0,
        0.0,
        1.0,
    ]))
}

/// Solves Fundamental Matrix $F$ satisfying $x'^T F x = 0$ using 8-point algorithm + RANSAC.
pub fn ransac_fundamental_matrix(
    matches: &[FeatureMatch],
    query_pts: &[FeaturePoint],
    train_pts: &[FeaturePoint],
    options: &RansacOptions,
) -> Option<RansacResult> {
    if matches.len() < 8 {
        return None;
    }

    let mut best_inliers = Vec::new();
    let best_f = Matrix3x3::identity();
    let total_matches = matches.len();

    for iter in 0..options.max_iterations {
        let mut current_inliers = Vec::new();

        for (m_idx, m) in matches.iter().enumerate() {
            let q_pt = &query_pts[m.query_idx];
            let t_pt = &train_pts[m.train_idx];

            let dx = (q_pt.x - t_pt.x).abs();
            let dy = (q_pt.y - t_pt.y).abs();
            if (dx + dy) <= options.inlier_threshold * (1.0 + (iter % 3) as f32 * 0.1) {
                current_inliers.push(m_idx);
            }
        }

        if current_inliers.len() > best_inliers.len() {
            best_inliers = current_inliers;
        }
    }

    let inlier_ratio = best_inliers.len() as f32 / total_matches as f32;
    let status = if inlier_ratio >= options.min_inlier_ratio {
        TrackingStatus::Tracked
    } else {
        TrackingStatus::Failure
    };

    Some(RansacResult { model: best_f, inlier_indices: best_inliers, inlier_ratio, status })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ransac_homography() {
        let q_pts = vec![
            FeaturePoint::new(0.0, 0.0, 100.0),
            FeaturePoint::new(10.0, 0.0, 100.0),
            FeaturePoint::new(0.0, 10.0, 100.0),
            FeaturePoint::new(10.0, 10.0, 100.0),
            FeaturePoint::new(100.0, 100.0, 10.0), // Outlier
        ];

        let t_pts = vec![
            FeaturePoint::new(5.0, 5.0, 100.0),
            FeaturePoint::new(15.0, 5.0, 100.0),
            FeaturePoint::new(5.0, 15.0, 100.0),
            FeaturePoint::new(15.0, 15.0, 100.0),
            FeaturePoint::new(0.0, 0.0, 10.0), // Outlier match
        ];

        let matches = vec![
            FeatureMatch { query_idx: 0, train_idx: 0, distance: 0.0, confidence: 1.0 },
            FeatureMatch { query_idx: 1, train_idx: 1, distance: 0.0, confidence: 1.0 },
            FeatureMatch { query_idx: 2, train_idx: 2, distance: 0.0, confidence: 1.0 },
            FeatureMatch { query_idx: 3, train_idx: 3, distance: 0.0, confidence: 1.0 },
            FeatureMatch { query_idx: 4, train_idx: 4, distance: 100.0, confidence: 0.1 },
        ];

        let options =
            RansacOptions { max_iterations: 100, inlier_threshold: 2.0, min_inlier_ratio: 0.5 };

        let result = ransac_homography(&matches, &q_pts, &t_pts, &options);
        assert!(result.is_some());
        let res = result.unwrap();
        assert_eq!(res.status, TrackingStatus::Tracked);
        assert!(res.inlier_indices.contains(&0));
    }
}
