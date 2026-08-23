//! Quality metrics, reprojection error diagnostics, and confidence scoring for 3D reconstruction.

use crate::cloud::PointCloud;

/// Quality assessment metrics for a 3D point cloud reconstruction segment.
#[derive(Debug, Clone, PartialEq)]
pub struct ReconstructionQuality {
    /// Average 2D pixel reprojection error across all triangulation matches
    pub mean_reprojection_error: f64,
    /// Spatial geometric consistency score [0.0, 1.0]
    pub geometric_consistency: f64,
    /// Frame-to-frame temporal stability score [0.0, 1.0]
    pub temporal_stability: f64,
    /// Combined confidence score [0.0, 1.0]
    pub overall_confidence: f64,
}

impl ReconstructionQuality {
    /// Computes quality metrics for a PointCloud and set of reprojection errors.
    pub fn evaluate(cloud: &PointCloud, reprojection_errors: &[f64]) -> Self {
        if cloud.is_empty() || reprojection_errors.is_empty() {
            return Self {
                mean_reprojection_error: 0.0,
                geometric_consistency: 0.0,
                temporal_stability: 0.0,
                overall_confidence: 0.0,
            };
        }

        let total_err: f64 = reprojection_errors.iter().sum();
        let mean_err = total_err / reprojection_errors.len() as f64;

        // Reprojection score: 1.0 at 0 error, degrades linearly up to 3.0 pixels
        let reproj_score = (1.0 - (mean_err / 3.0)).clamp(0.0, 1.0);

        // Calculate average point confidence
        let sum_conf: f32 = cloud.points.iter().map(|p| p.confidence).sum();
        let geom_score = (sum_conf as f64 / cloud.len() as f64).clamp(0.0, 1.0);

        // Estimate temporal stability based on mean observation count per point
        let sum_obs: u32 = cloud.points.iter().map(|p| p.observation_count).sum();
        let mean_obs = sum_obs as f64 / cloud.len() as f64;
        let temp_score = (mean_obs / 5.0).clamp(0.0, 1.0);

        let overall = (0.4 * reproj_score + 0.4 * geom_score + 0.2 * temp_score).clamp(0.0, 1.0);

        Self {
            mean_reprojection_error: mean_err,
            geometric_consistency: geom_score,
            temporal_stability: temp_score,
            overall_confidence: overall,
        }
    }
}

/// Structural diagnostics report for 3D reconstruction.
#[derive(Debug, Clone, PartialEq)]
pub struct DiagnosticsReport {
    /// Total number of 3D spatial points
    pub total_points: usize,
    /// Average point confidence
    pub average_confidence: f32,
    /// Quality evaluation summary
    pub quality: ReconstructionQuality,
}

impl DiagnosticsReport {
    /// Generates a diagnostics report for a point cloud and reprojection metrics.
    pub fn generate(cloud: &PointCloud, reprojection_errors: &[f64]) -> Self {
        let quality = ReconstructionQuality::evaluate(cloud, reprojection_errors);
        let sum_conf: f32 = cloud.points.iter().map(|p| p.confidence).sum();
        let avg_conf = if !cloud.is_empty() { sum_conf / cloud.len() as f32 } else { 0.0 };

        Self { total_points: cloud.len(), average_confidence: avg_conf, quality }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Point3D;
    use voxora_math::Vector3;

    #[test]
    fn test_reconstruction_quality_evaluation() {
        let mut cloud = PointCloud::new();
        cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 1.0), [255, 0, 0], 0.9));
        cloud.push(Point3D::new(Vector3::new(1.0, 0.0, 1.0), [0, 255, 0], 0.8));

        let report = DiagnosticsReport::generate(&cloud, &[0.5, 0.7]);
        assert_eq!(report.total_points, 2);
        assert!(report.quality.overall_confidence > 0.5);
    }
}
