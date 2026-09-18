//! Contains the math necessary for the skeleton solver.
//!
//! At a high level, the skeleton has inputs and outputs. Inputs are things like
//! [`Edge::input_rot_g`], which is an (optional) constraint on the rotation of an
//! [`Edge`]. Outputs are things like [`Edge::output_rot_g`], which needs to be solved
//! for.
//!
//! The goal of the solver is to solve for all the outputs, using the inputs. For more
//! info, see the [`skeleton`](crate::skeleton) module.

use std::collections::{HashSet, VecDeque};

use nalgebra::Vector3;
use petgraph::graph::{EdgeIndex, NodeIndex};

use crate::newtypes::Global;
use crate::skeleton::Graph;
use crate::{BoneKind, Point, Skeleton, UnitQuat};

impl Skeleton {
	/// Solves for the outputs of the skeletal model.
	///
	/// For more info on the algorithm, see [`crate::skeleton`].
	pub fn solve(&mut self) -> Result<(), SolveError> {
		// Pure 3DoF tracking has no positional input, so anchor the head joint at
		// the origin to give the breadth-first traversal a root.
		if self.find_root_nodes().next().is_none() {
			let root = self.root_node();
			self.graph[root].input_pos_g = Some(Global(Point::origin()));
		}

		// Breadth-first traversal that tracks the *parent edge* of each dequeued
		// node, so a child edge can inherit its parent's solved rotation.
		let mut queue: VecDeque<(NodeIndex, Option<EdgeIndex>)> =
			self.find_root_nodes().map(|n| (n, None)).collect();
		let mut visited_edges = HashSet::with_capacity(self.graph.edge_count());
		let mut visited_nodes = HashSet::with_capacity(self.graph.node_count());
		for &(n, _) in &queue {
			visited_nodes.insert(n);
		}

		while let Some((popped, parent_edge)) = queue.pop_front() {
			let parent_rot = parent_edge
				.map(|e| self.graph[e].output_rot_g.0)
				.unwrap_or_else(UnitQuat::identity);
			let parent_pos = self.graph[popped].output_pos_g.0;

			let mut neighbors = self.graph.neighbors(popped).detach();
			while let Some((edge, node)) = neighbors.next(&self.graph) {
				if visited_edges.contains(&edge) {
					continue;
				}
				visited_edges.insert(edge);

				do_fk(&mut self.graph, parent_rot, parent_pos, edge, node);

				if visited_nodes.insert(node) {
					queue.push_back((node, Some(edge)));
				}
			}
		}
		Ok(())
	}

	/// The head joint: the source endpoint of the `Neck` bone edge.
	pub(crate) fn root_node(&self) -> NodeIndex {
		let neck = self.bone_map[BoneKind::Neck];
		self.graph.edge_endpoints(neck).unwrap().0
	}
}

/// Solves one `edge` (rotation) and its child `node` (position) by applying
/// forward kinematics from the already-solved parent, whose rotation is `parent_rot`
/// and whose position is `parent_pos`.
fn do_fk(g: &mut Graph, parent_rot: UnitQuat, parent_pos: Point, edge: EdgeIndex, node: NodeIndex) {
	// 1. Solve the edge's global rotation.
	//
	// If a tracker pins this edge, copy its rotation. Otherwise the bone stays
	// rigid relative to its parent: inherit the parent's rotation composed with
	// the calibration offset that maps the parent frame to this bone's frame.
	let (input_rot, calib_rot, length) = {
		let e = &g[edge];
		(e.input_rot_g.as_ref().map(|r| r.0), e.calib_rot_l.0, e.length)
	};
	let rot = input_rot.unwrap_or_else(|| parent_rot * calib_rot);
	g[edge].output_rot_g = Global(rot);

	// 2. Solve the child node's global position.
	//
	// A pinned position wins; otherwise the child sits `length` below the parent
	// along the edge's (rotated) down axis. At calibration the bones point "up"
	// (toward the head/root), so the child is offset along -Y.
	let pinned = g[node].input_pos_g.as_ref().map(|p| p.0);
	match pinned {
		Some(pos) => g[node].output_pos_g = Global(pos),
		None => {
			let offset = rot * Vector3::new(0.0, -length, 0.0);
			g[node].output_pos_g = Global(parent_pos + offset);
		}
	}
}

#[derive(thiserror::Error, Debug)]
pub enum SolveError {
	#[error("Need at least one \"root\" `Node` (root nodes have a `input_pos_g`)")]
	NoRootNode,
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::skeleton::SkeletonConfig;
	use crate::BoneMap;
	use approx::assert_relative_eq;

	/// With unit bone lengths and no trackers, the FK produces a straight
	/// vertical chain: each joint sits one unit below its parent.
	#[test]
	fn fk_positions_identity() {
		let lengths = BoneMap::new([1.0; BoneKind::NUM_TYPES]);
		let mut s = Skeleton::new(&SkeletonConfig::new(lengths));
		s.solve().unwrap();

		assert_relative_eq!(s.bone_output_pos(BoneKind::Neck), Point::new(0.0, -1.0, 0.0));
		assert_relative_eq!(s.bone_output_pos(BoneKind::UpperChest), Point::new(0.0, -2.0, 0.0));
		assert_relative_eq!(s.bone_output_pos(BoneKind::Chest), Point::new(0.0, -3.0, 0.0));
		assert_relative_eq!(s.bone_output_pos(BoneKind::Waist), Point::new(0.0, -4.0, 0.0));
		assert_relative_eq!(s.bone_output_pos(BoneKind::Hip), Point::new(0.0, -5.0, 0.0));
	}

	/// A pinned chest rotation propagates to the untracked descendants.
	#[test]
	fn fk_pinned_rotation_inherits() {
		let lengths = BoneMap::new([1.0; BoneKind::NUM_TYPES]);
		let mut s = Skeleton::new(&SkeletonConfig::new(lengths));
		// Rotate the chest 90° about X; the hip should inherit it (no tracker).
		let q = UnitQuat::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_2);
		s.attach_input_tracker(BoneKind::Chest, [q.w, q.i, q.j, q.k]);
		s.solve().unwrap();

		let [cw, ci, cj, ck] = s.bone_output_rot(BoneKind::Chest);
		let chest = UnitQuat::from_quaternion(nalgebra::Quaternion::new(cw, ci, cj, ck));
		assert_relative_eq!(chest.angle_to(&q), 0.0, epsilon = 1e-5);

		// Untracked hip inherits chest's rotation (calib offset is identity here).
		let [hw, hi, hj, hk] = s.bone_output_rot(BoneKind::Hip);
		let hip = UnitQuat::from_quaternion(nalgebra::Quaternion::new(hw, hi, hj, hk));
		assert_relative_eq!(hip.angle_to(&q), 0.0, epsilon = 1e-5);
	}
}
