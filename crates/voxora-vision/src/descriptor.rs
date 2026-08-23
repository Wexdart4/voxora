//! Feature descriptors (Patch-based, 256-bit Binary BRIEF) and distance metrics.

use crate::FeaturePoint;
use voxora_core::Frame;

/// Normalized N x N patch feature descriptor.
#[derive(Debug, Clone, PartialEq)]
pub struct PatchDescriptor {
    /// Patch dimension (e.g. 7 for 7x7)
    pub size: usize,
    /// Normalized patch pixel intensities [0.0, 1.0]
    pub values: Vec<f32>,
}

/// 256-bit binary descriptor for fast Hamming distance matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BinaryDescriptor {
    /// 32 bytes representing 256 binary test bits
    pub bits: [u8; 32],
}

impl BinaryDescriptor {
    /// Zero binary descriptor.
    pub const ZERO: Self = Self { bits: [0u8; 32] };
}

/// Feature Descriptor enum wrapper.
#[derive(Debug, Clone, PartialEq)]
pub enum Descriptor {
    /// Intensity Patch descriptor
    Patch(PatchDescriptor),
    /// 256-bit Binary descriptor
    Binary(BinaryDescriptor),
}

/// Computes Hamming distance (number of differing bits) between two 256-bit binary descriptors.
pub fn hamming_distance(a: &BinaryDescriptor, b: &BinaryDescriptor) -> u32 {
    let mut dist = 0u32;
    for i in 0..32 {
        dist += (a.bits[i] ^ b.bits[i]).count_ones();
    }
    dist
}

/// Computes Euclidean distance between two patch descriptors.
pub fn euclidean_distance(a: &PatchDescriptor, b: &PatchDescriptor) -> f32 {
    if a.size != b.size || a.values.len() != b.values.len() {
        return f32::MAX;
    }
    let mut sum_sq = 0.0f32;
    for (v1, v2) in a.values.iter().zip(b.values.iter()) {
        let diff = v1 - v2;
        sum_sq += diff * diff;
    }
    sum_sq.sqrt()
}

/// Extracts a patch descriptor around a feature point.
pub fn compute_patch_descriptor(
    frame: &Frame,
    point: &FeaturePoint,
    patch_size: usize,
) -> Option<PatchDescriptor> {
    let gray = frame.to_grayscale();
    let width = gray.width as i32;
    let height = gray.height as i32;
    let radius = (patch_size / 2) as i32;

    let cx = point.x.round() as i32;
    let cy = point.y.round() as i32;

    if cx - radius < 0 || cx + radius >= width || cy - radius < 0 || cy + radius >= height {
        return None;
    }

    let mut values = Vec::with_capacity(patch_size * patch_size);

    for y in (cy - radius)..=(cy + radius) {
        for x in (cx - radius)..=(cx + radius) {
            let idx = (y * width + x) as usize;
            values.push(gray.data[idx] as f32 / 255.0);
        }
    }

    Some(PatchDescriptor { size: patch_size, values })
}

/// Computes a 256-bit binary BRIEF-style descriptor using deterministic random pixel pair tests.
pub fn compute_binary_descriptor(frame: &Frame, point: &FeaturePoint) -> Option<BinaryDescriptor> {
    let gray = frame.to_grayscale();
    let width = gray.width as i32;
    let height = gray.height as i32;

    let cx = point.x.round() as i32;
    let cy = point.y.round() as i32;

    let patch_radius = 12;
    if cx - patch_radius < 0
        || cx + patch_radius >= width
        || cy - patch_radius < 0
        || cy + patch_radius >= height
    {
        return None;
    }

    let mut bits = [0u8; 32];

    for i in 0..256 {
        // Deterministic offset generation for reproducibility
        let dx1 = ((i * 7 + 3) % 23) as i32 - 11;
        let dy1 = ((i * 11 + 5) % 23) as i32 - 11;
        let dx2 = ((i * 13 + 7) % 23) as i32 - 11;
        let dy2 = ((i * 17 + 9) % 23) as i32 - 11;

        let val1 = gray.data[((cy + dy1) * width + (cx + dx1)) as usize];
        let val2 = gray.data[((cy + dy2) * width + (cx + dx2)) as usize];

        if val1 < val2 {
            let byte_idx = i / 8;
            let bit_idx = i % 8;
            bits[byte_idx] |= 1 << bit_idx;
        }
    }

    Some(BinaryDescriptor { bits })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hamming_distance() {
        let b1 = BinaryDescriptor { bits: [0u8; 32] };
        let mut b2 = BinaryDescriptor { bits: [0u8; 32] };
        b2.bits[0] = 0b0000_1111; // 4 bit diffs

        assert_eq!(hamming_distance(&b1, &b2), 4);
    }

    #[test]
    fn test_euclidean_distance() {
        let p1 = PatchDescriptor { size: 2, values: vec![0.0, 0.0, 0.0, 0.0] };
        let p2 = PatchDescriptor { size: 2, values: vec![1.0, 0.0, 0.0, 0.0] };

        assert!((euclidean_distance(&p1, &p2) - 1.0).abs() < 1e-4);
    }
}
