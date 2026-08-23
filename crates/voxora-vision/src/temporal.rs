//! Temporal stabilization and moving-average trajectory/geometry smoothing.

use voxora_math::Vector3;

/// Temporal stabilizer for smoothing 3D spatial point positions over consecutive frames.
#[derive(Debug, Clone)]
pub struct TemporalStabilizer {
    /// Exponential smoothing weight factor $\alpha \in [0.0, 1.0]$
    pub alpha: f64,
}

impl Default for TemporalStabilizer {
    fn default() -> Self {
        Self { alpha: 0.7 }
    }
}

impl TemporalStabilizer {
    /// Creates a temporal stabilizer with given smoothing factor $\alpha$.
    pub fn new(alpha: f64) -> Self {
        Self { alpha: alpha.clamp(0.01, 1.0) }
    }

    /// Smooths a newly observed 3D position $P_{new}$ given previous position $P_{prev}$.
    pub fn smooth_point(&self, prev_pos: Vector3, new_pos: Vector3) -> Vector3 {
        Vector3::new(
            self.alpha * new_pos.x + (1.0 - self.alpha) * prev_pos.x,
            self.alpha * new_pos.y + (1.0 - self.alpha) * prev_pos.y,
            self.alpha * new_pos.z + (1.0 - self.alpha) * prev_pos.z,
        )
    }

    /// Batch smooths a sequence of 3D point positions.
    pub fn smooth_sequence(&self, positions: &[Vector3]) -> Vec<Vector3> {
        if positions.is_empty() {
            return Vec::new();
        }

        let mut smoothed = Vec::with_capacity(positions.len());
        smoothed.push(positions[0]);

        for i in 1..positions.len() {
            let next_pos = self.smooth_point(smoothed[i - 1], positions[i]);
            smoothed.push(next_pos);
        }

        smoothed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_temporal_smoothing() {
        let stabilizer = TemporalStabilizer::new(0.5);
        let p1 = Vector3::new(0.0, 0.0, 0.0);
        let p2 = Vector3::new(10.0, 0.0, 0.0);
        let smoothed = stabilizer.smooth_point(p1, p2);

        assert!((smoothed.x - 5.0).abs() < 1e-4);
    }
}
