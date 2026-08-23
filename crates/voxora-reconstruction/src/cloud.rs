//! Point Cloud container, voxel grid spatial downsampling, statistical outlier removal, and duplicate merging.

use voxora_math::Vector3;

/// 3D point structure with spatial coordinates, RGB color, confidence score, and observation count.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3D {
    /// 3D position (X, Y, Z) in world coordinates
    pub position: Vector3,
    /// RGB color bytes [R, G, B]
    pub color: [u8; 3],
    /// Reconstruction confidence score [0.0, 1.0]
    pub confidence: f32,
    /// Number of frame observations contributing to point
    pub observation_count: u32,
    /// Source frame identifier
    pub frame_id: usize,
}

impl Point3D {
    /// Creates a 3D point.
    pub fn new(position: Vector3, color: [u8; 3], confidence: f32) -> Self {
        Self { position, color, confidence, observation_count: 1, frame_id: 0 }
    }
}

/// Point Cloud container storing spatial 3D points.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointCloud {
    /// Collection of 3D points
    pub points: Vec<Point3D>,
}

impl PointCloud {
    /// Creates a new empty point cloud container.
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends a point to the point cloud.
    pub fn push(&mut self, point: Point3D) {
        self.points.push(point);
    }

    /// Returns the number of points in the point cloud.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Returns whether the point cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Merges duplicate spatially adjacent points within a minimum Euclidean distance threshold $\epsilon$.
    pub fn merge_duplicate_points(&self, min_distance: f64) -> PointCloud {
        let mut merged: Vec<Point3D> = Vec::new();

        for pt in &self.points {
            let mut is_duplicate = false;

            for m in &mut merged {
                let dx = pt.position.x - m.position.x;
                let dy = pt.position.y - m.position.y;
                let dz = pt.position.z - m.position.z;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();

                if dist <= min_distance {
                    m.observation_count += 1;
                    m.confidence = ((m.confidence * (m.observation_count - 1) as f32)
                        + pt.confidence)
                        / m.observation_count as f32;
                    is_duplicate = true;
                    break;
                }
            }

            if !is_duplicate {
                merged.push(*pt);
            }
        }

        PointCloud { points: merged }
    }

    /// Downsamples point cloud using a 3D Voxel Grid spatial filter.
    pub fn voxel_grid_filter(&self, voxel_size: f64) -> PointCloud {
        if voxel_size <= 1e-6 || self.points.is_empty() {
            return self.clone();
        }

        use std::collections::HashMap;
        let mut grid: HashMap<(i64, i64, i64), Vec<Point3D>> = HashMap::new();

        for pt in &self.points {
            let vx = (pt.position.x / voxel_size).floor() as i64;
            let vy = (pt.position.y / voxel_size).floor() as i64;
            let vz = (pt.position.z / voxel_size).floor() as i64;

            grid.entry((vx, vy, vz)).or_default().push(*pt);
        }

        let mut filtered_points = Vec::with_capacity(grid.len());
        for (_cell, pts) in grid {
            let count = pts.len() as f64;
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut sum_z = 0.0;
            let mut sum_r = 0.0;
            let mut sum_g = 0.0;
            let mut sum_b = 0.0;
            let mut sum_conf = 0.0f32;

            for p in &pts {
                sum_x += p.position.x;
                sum_y += p.position.y;
                sum_z += p.position.z;
                sum_r += p.color[0] as f64;
                sum_g += p.color[1] as f64;
                sum_b += p.color[2] as f64;
                sum_conf += p.confidence;
            }

            let avg_pos = Vector3::new(sum_x / count, sum_y / count, sum_z / count);
            let avg_color = [(sum_r / count) as u8, (sum_g / count) as u8, (sum_b / count) as u8];
            let avg_conf = sum_conf / count as f32;

            let mut avg_pt = Point3D::new(avg_pos, avg_color, avg_conf);
            avg_pt.observation_count = pts.len() as u32;
            filtered_points.push(avg_pt);
        }

        PointCloud { points: filtered_points }
    }

    /// Removes isolated outlier points whose average k-nearest neighbor distance exceeds $\mu + k \cdot \sigma$.
    pub fn statistical_outlier_removal(&self, k_neighbors: usize, std_mul: f64) -> PointCloud {
        let n = self.points.len();
        if n <= k_neighbors || k_neighbors == 0 {
            return self.clone();
        }

        let mut mean_distances = vec![0.0f64; n];

        for (i, mean_dist) in mean_distances.iter_mut().enumerate().take(n) {
            let p1 = &self.points[i];
            let mut dists = Vec::with_capacity(n - 1);

            for (j, p2) in self.points.iter().enumerate() {
                if i != j {
                    let dx = p1.position.x - p2.position.x;
                    let dy = p1.position.y - p2.position.y;
                    let dz = p1.position.z - p2.position.z;
                    dists.push((dx * dx + dy * dy + dz * dz).sqrt());
                }
            }

            dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let k_sum: f64 = dists.iter().take(k_neighbors).sum();
            *mean_dist = k_sum / k_neighbors as f64;
        }

        let global_mean: f64 = mean_distances.iter().sum::<f64>() / n as f64;
        let variance: f64 =
            mean_distances.iter().map(|d| (d - global_mean).powi(2)).sum::<f64>() / n as f64;
        let std_dev = variance.sqrt();

        let threshold = global_mean + std_mul * std_dev;
        let mut filtered = Vec::new();

        for (i, pt) in self.points.iter().enumerate() {
            if mean_distances[i] <= threshold {
                filtered.push(*pt);
            }
        }

        PointCloud { points: filtered }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_cloud_filtering() {
        let mut cloud = PointCloud::new();
        cloud.push(Point3D::new(Vector3::new(0.0, 0.0, 0.0), [255, 0, 0], 1.0));
        cloud.push(Point3D::new(Vector3::new(0.01, 0.01, 0.01), [255, 0, 0], 1.0));
        cloud.push(Point3D::new(Vector3::new(10.0, 10.0, 10.0), [0, 255, 0], 1.0));

        let voxel_filtered = cloud.voxel_grid_filter(0.1);
        assert_eq!(voxel_filtered.len(), 2);

        let merged = cloud.merge_duplicate_points(0.05);
        assert_eq!(merged.len(), 2);
    }
}
