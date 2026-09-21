use num_derive::{FromPrimitive, ToPrimitive};
use num_traits::FromPrimitive;

use crate::{
	conventions::{forward_vec, up_vec},
	newtypes::{Global, Local},
	UnitQuat,
};

#[allow(rustdoc::private_intra_doc_links)]
/// Describes the various types of bones in the skeleton.
///
/// All of this information is static rather than dynamic information. "Static" here
/// refers to information that does not change - this information is "hard coded".
/// This includes:
/// - The parent/child relationships between the various types of bones
/// - The pose of the bones when performing a calibration
/// - Names of the bones
/// - Etc
///
/// Where possible, this information is provided as [`const`] functions so that they
/// can be evaluated at compile-time.
///
/// There is also dynamic information associated with bones. "Dynamic" here refers to
/// the fact that some bone data cannot be known up front and changes as the skeletal
/// model receives inputs and produces outputs. This data is stored in the
/// [`Skeleton`](crate::Skeleton) as an [`Edge`](crate::skeleton::Edge).
///
/// `BoneKind` is also represented as a `u8`, so it can be used as an index for an
/// array. This is used for example in [`BoneMap`]. **Please note that we make no
/// stability guarantees for the particular value that any variant gets, only that
/// these values are contiguous and start at 0.** Use the variant directly or refer to
/// the various functions implemented on this type for stability.
///
/// [`const`]: https://doc.rust-lang.org/std/keyword.const.html
/// [`BoneMap`]: super::BoneMap
#[repr(u8)]
#[derive(Debug, Clone, Copy, Eq, Hash, PartialEq, FromPrimitive, ToPrimitive)]
pub enum BoneKind {
	Neck = 0,
	UpperChest,
	Chest,
	Waist,
	Hip,
	HipL,
	HipR,
	ThighL,
	ThighR,
	AnkleL,
	AnkleR,
	FootL,
	FootR,

	ShoulderL,
	ShoulderR,
	UpperArmL,
	UpperArmR,
	ForearmL,
	ForearmR,
	WristL,
	WristR,
}
impl BoneKind {
	/// The bone with the largest integer value
	pub const fn max() -> BoneKind {
		BoneKind::WristR
	}
	pub const MAX: BoneKind = Self::max();

	/// The bone with the smallest integer value
	pub const fn min() -> BoneKind {
		BoneKind::root()
	}
	pub const MIN: BoneKind = Self::min();

	/// The root bone of the skeletal graph/tree.
	pub const fn root() -> Self {
		Self::Neck
	}
	pub const ROOT: BoneKind = Self::root();

	/// Returns the number of unique kinds of bones. This is equivalent to the number
	/// of variants in `BoneKind`
	pub const fn num_types() -> usize {
		BoneKind::max() as usize + 1
	}
	pub const NUM_TYPES: usize = Self::num_types();

	/// Returns the children of any particular bone.
	///
	/// The slice is `'static`, which means the lifetime of the returned slice lives
	/// for the entire duration of the program. This is because the parent/child
	/// relationship of bones is known at compile-time.
	pub const fn children(&self) -> &'static [Self] {
		use BoneKind::*;
		match self {
			Neck => &[UpperChest],
			UpperChest => &[Chest, ShoulderL, ShoulderR],
			Chest => &[Waist],
			Waist => &[Hip],
			Hip => &[HipL, HipR],
			HipL => &[ThighL],
			HipR => &[ThighR],
			ThighL => &[AnkleL],
			ThighR => &[AnkleR],
			AnkleL => &[FootL],
			AnkleR => &[FootR],
			FootL => &[],
			FootR => &[],

			ShoulderL => &[UpperArmL],
			ShoulderR => &[UpperArmR],
			UpperArmL => &[ForearmL],
			UpperArmR => &[ForearmR],
			ForearmL => &[WristL],
			ForearmR => &[WristR],
			WristR => &[],
			WristL => &[],
		}
	}

	/// The parent of a bone.
	pub const fn parent(&self) -> Option<BoneKind> {
		use BoneKind::*;
		Some(match self {
			Neck => return None,
			UpperChest => Neck,
			Chest => UpperChest,
			Waist => Chest,
			Hip => Waist,
			HipL => Hip,
			HipR => Hip,
			ThighL => HipL,
			ThighR => HipR,
			AnkleL => ThighL,
			AnkleR => ThighR,
			FootL => AnkleL,
			FootR => AnkleR,

			ShoulderL => UpperChest,
			ShoulderR => UpperChest,
			UpperArmL => ShoulderL,
			UpperArmR => ShoulderR,
			ForearmL => UpperArmL,
			ForearmR => UpperArmR,
			WristL => ForearmL,
			WristR => ForearmR,
		})
	}

	pub fn iter() -> std::iter::Map<std::ops::RangeInclusive<u8>, fn(u8) -> BoneKind> {
		(Self::MIN as u8..=Self::MAX as u8).map(|x| x.try_into().unwrap())
	}

	/// Returns the initial calibration pose of the bone. Rotating the up vector by
	/// this rotation would cause it to point in the same target direction as the bone.
	pub fn calibration_rotation(self) -> Global<UnitQuat> {
		use BoneKind::*;
		Global(match self {
			// The solver places children along rotated -Y. These connector bones
			// must run sideways so left/right limbs have distinct attachment points.
			ShoulderL | HipL => UnitQuat::from_axis_angle(
				&nalgebra::Vector3::z_axis(), -std::f32::consts::FRAC_PI_2),
			ShoulderR | HipR => UnitQuat::from_axis_angle(
				&nalgebra::Vector3::z_axis(), std::f32::consts::FRAC_PI_2),
			FootL | FootR => UnitQuat::look_at_rh(&-up_vec(), &forward_vec()),
			_ => UnitQuat::default(),
		})
	}

	/// Returns the initial calibration pose of the bone, as a rotation relative to the
	/// parent bone. See also: [`Self::calibration_rotation`]
	pub fn calibration_rotation_local(self) -> Local<UnitQuat> {
		let child_rot_g = self.calibration_rotation();
		let parent_rot_g = self.parent().unwrap_or(self).calibration_rotation();
		Local(parent_rot_g.0.rotation_to(&child_rot_g.0))
	}
}
impl TryFrom<u8> for BoneKind {
	type Error = ();

	fn try_from(value: u8) -> Result<Self, Self::Error> {
		FromPrimitive::from_u8(value).ok_or(())
	}
}
impl TryFrom<usize> for BoneKind {
	type Error = ();
	fn try_from(value: usize) -> Result<Self, Self::Error> {
		FromPrimitive::from_usize(value).ok_or(())
	}
}
impl From<BoneKind> for u8 {
	fn from(other: BoneKind) -> Self {
		other as _
	}
}
impl From<BoneKind> for usize {
	fn from(other: BoneKind) -> Self {
		other as _
	}
}
