//! Indexation Spatiale Octree Hiérarchique 3D / LOD pour Données Photogrammétriques Massives
//!
//! Permet le découpage spatial, le culling d'emprise (Bounding Box) et la sélection de niveau de détail (LOD)
//! pour traiter et afficher dynamiquement des nuages de points de plusieurs dizaines de millions de points.

use serde::{Deserialize, Serialize};

/// Point 3D géoréférencé avec précision `f64` et couleur RGB.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpatialPoint3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl SpatialPoint3D {
    pub fn new(x: f64, y: f64, z: f64, r: u8, g: u8, b: u8) -> Self {
        Self { x, y, z, r, g, b }
    }
}

/// Boîte englobante 3D (Axis-Aligned Bounding Box - AABB) en précision `f64`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox3D {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

impl BoundingBox3D {
    pub fn new(min_x: f64, min_y: f64, min_z: f64, max_x: f64, max_y: f64, max_z: f64) -> Self {
        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
        }
    }

    pub fn from_points(points: &[SpatialPoint3D]) -> Self {
        if points.is_empty() {
            return Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut min_z = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut max_z = f64::NEG_INFINITY;

        for p in points {
            if p.x < min_x { min_x = p.x; }
            if p.y < min_y { min_y = p.y; }
            if p.z < min_z { min_z = p.z; }
            if p.x > max_x { max_x = p.x; }
            if p.y > max_y { max_y = p.y; }
            if p.z > max_z { max_z = p.z; }
        }

        Self::new(min_x, min_y, min_z, max_x, max_y, max_z)
    }

    #[inline]
    pub fn contains(&self, p: &SpatialPoint3D) -> bool {
        p.x >= self.min_x && p.x <= self.max_x
            && p.y >= self.min_y && p.y <= self.max_y
            && p.z >= self.min_z && p.z <= self.max_z
    }

    #[inline]
    pub fn intersects(&self, other: &BoundingBox3D) -> bool {
        self.min_x <= other.max_x && self.max_x >= other.min_x
            && self.min_y <= other.max_y && self.max_y >= other.min_y
            && self.min_z <= other.max_z && self.max_z >= other.min_z
    }

    pub fn center(&self) -> (f64, f64, f64) {
        (
            (self.min_x + self.max_x) * 0.5,
            (self.min_y + self.max_y) * 0.5,
            (self.min_z + self.max_z) * 0.5,
        )
    }
}

/// Nœud d'Octree 3D.
#[derive(Debug, Clone)]
pub struct OctreeNode {
    pub bounds: BoundingBox3D,
    pub points: Vec<SpatialPoint3D>,
    pub children: Option<Box<[OctreeNode; 8]>>,
}

impl OctreeNode {
    pub fn new(bounds: BoundingBox3D) -> Self {
        Self {
            bounds,
            points: Vec::new(),
            children: None,
        }
    }

    pub fn insert(&mut self, point: SpatialPoint3D, max_points_per_node: usize, max_depth: usize, depth: usize) {
        if !self.bounds.contains(&point) {
            return;
        }

        if depth >= max_depth || (self.children.is_none() && self.points.len() < max_points_per_node) {
            self.points.push(point);
            return;
        }

        if self.children.is_none() {
            self.subdivide();
            let existing = std::mem::take(&mut self.points);
            for p in existing {
                self.insert_to_children(p, max_points_per_node, max_depth, depth);
            }
        }

        self.insert_to_children(point, max_points_per_node, max_depth, depth);
    }

    fn insert_to_children(&mut self, point: SpatialPoint3D, max_points_per_node: usize, max_depth: usize, depth: usize) {
        if let Some(ref mut children) = self.children {
            for child in children.iter_mut() {
                if child.bounds.contains(&point) {
                    child.insert(point, max_points_per_node, max_depth, depth + 1);
                    break;
                }
            }
        }
    }

    fn subdivide(&mut self) {
        let (cx, cy, cz) = self.bounds.center();
        let b = &self.bounds;

        let octants = [
            BoundingBox3D::new(b.min_x, b.min_y, b.min_z, cx, cy, cz),
            BoundingBox3D::new(cx, b.min_y, b.min_z, b.max_x, cy, cz),
            BoundingBox3D::new(b.min_x, cy, b.min_z, cx, b.max_y, cz),
            BoundingBox3D::new(cx, cy, b.min_z, b.max_x, b.max_y, cz),
            BoundingBox3D::new(b.min_x, b.min_y, cz, cx, cy, b.max_z),
            BoundingBox3D::new(cx, b.min_y, cz, b.max_x, cy, b.max_z),
            BoundingBox3D::new(b.min_x, cy, cz, cx, b.max_y, b.max_z),
            BoundingBox3D::new(cx, cy, cz, b.max_x, b.max_y, b.max_z),
        ];

        self.children = Some(Box::new([
            OctreeNode::new(octants[0]),
            OctreeNode::new(octants[1]),
            OctreeNode::new(octants[2]),
            OctreeNode::new(octants[3]),
            OctreeNode::new(octants[4]),
            OctreeNode::new(octants[5]),
            OctreeNode::new(octants[6]),
            OctreeNode::new(octants[7]),
        ]));
    }

    pub fn query_bbox(&self, bbox: &BoundingBox3D, out: &mut Vec<SpatialPoint3D>) {
        if !self.bounds.intersects(bbox) {
            return;
        }

        for p in &self.points {
            if bbox.contains(p) {
                out.push(*p);
            }
        }

        if let Some(ref children) = self.children {
            for child in children.iter() {
                child.query_bbox(bbox, out);
            }
        }
    }

    pub fn query_lod(&self, bbox: &BoundingBox3D, target_max_points: usize, current_depth: usize, max_lod_depth: usize, out: &mut Vec<SpatialPoint3D>) {
        if !self.bounds.intersects(bbox) || out.len() >= target_max_points {
            return;
        }

        if current_depth >= max_lod_depth || self.children.is_none() {
            self.query_bbox(bbox, out);
            if out.len() > target_max_points {
                out.truncate(target_max_points);
            }
            return;
        }

        if let Some(ref children) = self.children {
            for child in children.iter() {
                child.query_lod(bbox, target_max_points, current_depth + 1, max_lod_depth, out);
                if out.len() >= target_max_points {
                    break;
                }
            }
        }
    }
}

/// Index Spatiale Octree Global pour Nuages de Points Massifs.
#[derive(Debug, Clone)]
pub struct SpatialOctree {
    pub root: Option<OctreeNode>,
    pub total_points: usize,
    pub max_points_per_leaf: usize,
    pub max_depth: usize,
}

impl SpatialOctree {
    pub fn new(max_points_per_leaf: usize, max_depth: usize) -> Self {
        Self {
            root: None,
            total_points: 0,
            max_points_per_leaf,
            max_depth,
        }
    }

    pub fn build(&mut self, points: &[SpatialPoint3D]) {
        if points.is_empty() {
            self.root = None;
            self.total_points = 0;
            return;
        }

        let bounds = BoundingBox3D::from_points(points);
        let mut root = OctreeNode::new(bounds);

        for &p in points {
            root.insert(p, self.max_points_per_leaf, self.max_depth, 0);
        }

        self.root = Some(root);
        self.total_points = points.len();
    }

    pub fn query_bounding_box(&self, bbox: &BoundingBox3D) -> Vec<SpatialPoint3D> {
        let mut result = Vec::new();
        if let Some(ref root) = self.root {
            root.query_bbox(bbox, &mut result);
        }
        result
    }

    pub fn query_lod(&self, bbox: &BoundingBox3D, target_max_points: usize, max_lod_depth: usize) -> Vec<SpatialPoint3D> {
        let mut result = Vec::with_capacity(target_max_points.min(100_000));
        if let Some(ref root) = self.root {
            root.query_lod(bbox, target_max_points, 0, max_lod_depth, &mut result);
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_octree_building_and_query() {
        let mut points = Vec::new();
        for i in 0..1000 {
            let val = i as f64;
            points.push(SpatialPoint3D::new(val, val * 2.0, val * 0.5, 255, 0, 0));
        }

        let mut octree = SpatialOctree::new(32, 6);
        octree.build(&points);

        assert_eq!(octree.total_points, 1000);

        let search_bbox = BoundingBox3D::new(10.0, 20.0, 5.0, 50.0, 100.0, 25.0);
        let queried = octree.query_bounding_box(&search_bbox);

        assert!(!queried.is_empty());
        for p in queried {
            assert!(search_bbox.contains(&p));
        }
    }

    #[test]
    fn test_lod_query_truncation() {
        let mut points = Vec::new();
        for i in 0..5000 {
            let val = i as f64;
            points.push(SpatialPoint3D::new(val, val, val, 0, 255, 0));
        }

        let mut octree = SpatialOctree::new(64, 5);
        octree.build(&points);

        let full_bbox = BoundingBox3D::new(0.0, 0.0, 0.0, 6000.0, 6000.0, 6000.0);
        let lod_points = octree.query_lod(&full_bbox, 250, 3);

        assert!(lod_points.len() <= 250);
        assert!(!lod_points.is_empty());
    }
}
