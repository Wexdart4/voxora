//! Scene cut detection, dynamic object segmentation, and motion isolation.

use voxora_core::{Frame, VoxoraError};
use voxora_math::Matrix3x3;

/// Types of detected video transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CutType {
    /// Continuous video stream without transition
    None,
    /// Sudden hard cut between unrelated scenes
    HardCut,
    /// Smooth fade-to-black or cross-fade transition
    Fade,
}

/// Scene cut detector monitoring video stream intensity shifts and tracking degradation.
#[derive(Debug, Clone)]
pub struct SceneCutDetector {
    /// Intensity histogram difference threshold for hard cut detection [0.0, 1.0]
    pub cut_threshold: f64,
    /// Minimum feature tracking match ratio threshold [0.0, 1.0]
    pub min_match_ratio: f64,
    /// Previous frame intensity histogram
    prev_histogram: Option<[f64; 16]>,
}

impl Default for SceneCutDetector {
    fn default() -> Self {
        Self { cut_threshold: 0.35, min_match_ratio: 0.2, prev_histogram: None }
    }
}

impl SceneCutDetector {
    /// Creates a new scene cut detector with custom thresholds.
    pub fn new(cut_threshold: f64, min_match_ratio: f64) -> Self {
        Self { cut_threshold, min_match_ratio, prev_histogram: None }
    }

    /// Computes a normalized 16-bin grayscale intensity histogram for a frame.
    fn compute_histogram(frame: &Frame) -> [f64; 16] {
        let gray = frame.to_grayscale();
        let mut hist = [0f64; 16];
        if gray.data.is_empty() {
            return hist;
        }

        for &val in &gray.data {
            let bin = (val as usize / 16).min(15);
            hist[bin] += 1.0;
        }

        let total = gray.data.len() as f64;
        for bin in &mut hist {
            *bin /= total;
        }

        hist
    }

    /// Evaluates if a new video frame represents a scene transition.
    pub fn detect_cut(
        &mut self,
        frame: &Frame,
        tracking_match_ratio: f64,
    ) -> Result<CutType, VoxoraError> {
        let curr_hist = Self::compute_histogram(frame);

        if let Some(prev_hist) = self.prev_histogram {
            // Compute Bhattacharyya distance / histogram difference
            let mut diff = 0.0;
            for i in 0..16 {
                diff += (curr_hist[i] - prev_hist[i]).abs();
            }

            self.prev_histogram = Some(curr_hist);

            if diff > self.cut_threshold || tracking_match_ratio < self.min_match_ratio {
                if diff > 0.6 {
                    return Ok(CutType::HardCut);
                } else if tracking_match_ratio < self.min_match_ratio {
                    return Ok(CutType::Fade);
                }
            }
        } else {
            self.prev_histogram = Some(curr_hist);
        }

        Ok(CutType::None)
    }

    /// Resets historical state.
    pub fn reset(&mut self) {
        self.prev_histogram = None;
    }
}

/// Dynamic object segmentation filter separating moving foreground objects from static background.
#[derive(Debug, Clone)]
pub struct DynamicMaskEstimator {
    /// Sampson epipolar error threshold for identifying dynamic outliers (in pixels)
    pub epipolar_threshold: f64,
}

impl Default for DynamicMaskEstimator {
    fn default() -> Self {
        Self { epipolar_threshold: 2.5 }
    }
}

impl DynamicMaskEstimator {
    /// Creates a dynamic mask estimator with specified epipolar threshold.
    pub fn new(epipolar_threshold: f64) -> Self {
        Self { epipolar_threshold }
    }

    /// Identifies static vs dynamic feature matches given estimated Fundamental/Essential matrix $F$.
    ///
    /// Returns boolean mask where `true` indicates a reliable static background point.
    pub fn filter_static_features(
        &self,
        start_points: &[(f64, f64)],
        end_points: &[(f64, f64)],
        fundamental_matrix: &Matrix3x3,
    ) -> Vec<bool> {
        let mut static_mask = Vec::with_capacity(start_points.len());

        for i in 0..start_points.len() {
            let p1 = voxora_math::Vector3::new(start_points[i].0, start_points[i].1, 1.0);
            let p2 = voxora_math::Vector3::new(end_points[i].0, end_points[i].1, 1.0);

            // Epipolar constraint p2^T * F * p1 ~ 0
            let f_p1 = fundamental_matrix.mul_vec(p1);
            let epipolar_err = (p2.x * f_p1.x + p2.y * f_p1.y + f_p1.z).abs();

            static_mask.push(epipolar_err < self.epipolar_threshold);
        }

        static_mask
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use voxora_core::PixelFormat;

    #[test]
    fn test_scene_cut_detection() {
        let mut detector = SceneCutDetector::new(0.3, 0.2);

        let frame1 = Frame::new(10, 10, PixelFormat::Grayscale, vec![0; 100]).unwrap();
        let cut1 = detector.detect_cut(&frame1, 1.0).unwrap();
        assert_eq!(cut1, CutType::None);

        let frame2 = Frame::new(10, 10, PixelFormat::Grayscale, vec![255; 100]).unwrap();
        let cut2 = detector.detect_cut(&frame2, 0.05).unwrap();
        assert_ne!(cut2, CutType::None);
    }
}
