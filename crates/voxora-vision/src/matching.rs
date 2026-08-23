//! Frame-to-frame feature matching, Lowe ratio test, mutual cross-check, and spatial filtering.

use crate::descriptor::{euclidean_distance, hamming_distance, Descriptor};
use crate::FeaturePoint;

/// Represents a correspondence match between a query feature point and a train feature point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeatureMatch {
    /// Index in the query feature point vector
    pub query_idx: usize,
    /// Index in the train feature point vector
    pub train_idx: usize,
    /// Descriptor distance (lower is better)
    pub distance: f32,
    /// Calculated match confidence score [0.0, 1.0]
    pub confidence: f32,
}

/// Brute-force descriptor matcher supporting distance metrics, ratio testing, and cross-checking.
#[derive(Debug, Clone)]
pub struct BruteForceMatcher {
    /// Lowe's ratio test threshold (typically 0.7 - 0.8)
    pub ratio_threshold: f32,
    /// Maximum spatial distance radius constraint (in pixels, optional)
    pub max_spatial_distance: Option<f32>,
    /// Require mutual cross-check (bidirectional best match)
    pub cross_check: bool,
}

impl Default for BruteForceMatcher {
    fn default() -> Self {
        Self { ratio_threshold: 0.8, max_spatial_distance: None, cross_check: true }
    }
}

impl BruteForceMatcher {
    /// Creates a new BruteForceMatcher with specified configuration.
    pub fn new(ratio_threshold: f32, max_spatial_distance: Option<f32>, cross_check: bool) -> Self {
        Self { ratio_threshold, max_spatial_distance, cross_check }
    }

    /// Matches query feature points against train feature points.
    pub fn match_features(
        &self,
        query: &[FeaturePoint],
        train: &[FeaturePoint],
    ) -> Vec<FeatureMatch> {
        if query.is_empty() || train.is_empty() {
            return Vec::new();
        }

        let mut matches = Vec::new();
        let mut best_train_for_query = vec![None; query.len()];

        for (q_idx, q_pt) in query.iter().enumerate() {
            let q_desc = match &q_pt.descriptor {
                Some(d) => d,
                None => continue,
            };

            let mut best_dist = f32::MAX;
            let mut second_best_dist = f32::MAX;
            let mut best_t_idx = None;

            for (t_idx, t_pt) in train.iter().enumerate() {
                // Spatial distance constraint
                if let Some(max_dist) = self.max_spatial_distance {
                    let dx = q_pt.x - t_pt.x;
                    let dy = q_pt.y - t_pt.y;
                    if (dx * dx + dy * dy).sqrt() > max_dist {
                        continue;
                    }
                }

                let t_desc = match &t_pt.descriptor {
                    Some(d) => d,
                    None => continue,
                };

                let dist = match (q_desc, t_desc) {
                    (Descriptor::Binary(b1), Descriptor::Binary(b2)) => {
                        hamming_distance(b1, b2) as f32
                    }
                    (Descriptor::Patch(p1), Descriptor::Patch(p2)) => euclidean_distance(p1, p2),
                    _ => continue,
                };

                if dist < best_dist {
                    second_best_dist = best_dist;
                    best_dist = dist;
                    best_t_idx = Some(t_idx);
                } else if dist < second_best_dist {
                    second_best_dist = dist;
                }
            }

            if let Some(t_idx) = best_t_idx {
                // Lowe's ratio test filter
                if second_best_dist > 0.0 && (best_dist / second_best_dist) <= self.ratio_threshold
                {
                    let confidence =
                        (1.0 - (best_dist / (second_best_dist + 1e-5))).clamp(0.0, 1.0);
                    best_train_for_query[q_idx] = Some((t_idx, best_dist, confidence));
                }
            }
        }

        if !self.cross_check {
            for (q_idx, match_opt) in best_train_for_query.into_iter().enumerate() {
                if let Some((t_idx, dist, confidence)) = match_opt {
                    matches.push(FeatureMatch {
                        query_idx: q_idx,
                        train_idx: t_idx,
                        distance: dist,
                        confidence,
                    });
                }
            }
            return matches;
        }

        // Cross-check: verify train -> query best match
        for (q_idx, match_opt) in best_train_for_query.into_iter().enumerate() {
            let (t_idx, dist, confidence) = match match_opt {
                Some(m) => m,
                None => continue,
            };

            let t_desc = match &train[t_idx].descriptor {
                Some(d) => d,
                None => continue,
            };

            let mut reverse_best_dist = f32::MAX;
            let mut reverse_best_q_idx = None;

            for (other_q_idx, other_q_pt) in query.iter().enumerate() {
                let other_q_desc = match &other_q_pt.descriptor {
                    Some(d) => d,
                    None => continue,
                };

                let rev_dist = match (t_desc, other_q_desc) {
                    (Descriptor::Binary(b1), Descriptor::Binary(b2)) => {
                        hamming_distance(b1, b2) as f32
                    }
                    (Descriptor::Patch(p1), Descriptor::Patch(p2)) => euclidean_distance(p1, p2),
                    _ => continue,
                };

                if rev_dist < reverse_best_dist {
                    reverse_best_dist = rev_dist;
                    reverse_best_q_idx = Some(other_q_idx);
                }
            }

            if reverse_best_q_idx == Some(q_idx) {
                matches.push(FeatureMatch {
                    query_idx: q_idx,
                    train_idx: t_idx,
                    distance: dist,
                    confidence,
                });
            }
        }

        matches
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::descriptor::BinaryDescriptor;

    #[test]
    fn test_brute_force_matching() {
        let mut pt1 = FeaturePoint::new(10.0, 10.0, 100.0);
        let mut desc1 = BinaryDescriptor { bits: [0u8; 32] };
        desc1.bits[0] = 0b1111_0000;
        pt1.descriptor = Some(Descriptor::Binary(desc1));

        let mut pt2 = FeaturePoint::new(12.0, 10.0, 90.0);
        let mut desc2 = BinaryDescriptor { bits: [0u8; 32] };
        desc2.bits[0] = 0b1111_0000; // Exact match
        pt2.descriptor = Some(Descriptor::Binary(desc2));

        let mut pt3 = FeaturePoint::new(50.0, 50.0, 80.0);
        let mut desc3 = BinaryDescriptor { bits: [0u8; 32] };
        desc3.bits[0] = 0b0000_1111; // Distant match
        pt3.descriptor = Some(Descriptor::Binary(desc3));

        let query = vec![pt1];
        let train = vec![pt2, pt3];

        let matcher = BruteForceMatcher::new(0.9, None, true);
        let matches = matcher.match_features(&query, &train);

        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].query_idx, 0);
        assert_eq!(matches[0].train_idx, 0); // Matched exact desc2
        assert_eq!(matches[0].distance, 0.0);
    }
}
