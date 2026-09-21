//! OSC-JSON avatar parameter mapping, ported from oscavmgr's
//! `src/core/ext_oscjson.rs` (`OscJsonNode` + `MysteryParam`).

use rosc::{OscBundle, OscType};
use serde::Deserialize;
use std::collections::HashMap;
use std::str::FromStr;
use std::sync::Arc;

use super::bundle::AvatarBundle;
use super::unified::{CombinedExpression, UnifiedExpressions, NUM_SHAPES};

/// A node of the OSC-JSON tree returned by VRChat.
#[derive(Deserialize, Debug)]
pub struct OscJsonNode {
    #[serde(alias = "FULL_PATH")]
    pub full_path: Arc<str>,
    #[serde(default, alias = "ACCESS")]
    pub access: i32,
    #[serde(alias = "TYPE")]
    pub data_type: Option<Arc<str>>,
    #[serde(alias = "CONTENTS")]
    pub contents: Option<HashMap<Arc<str>, OscJsonNode>>,
}

impl OscJsonNode {
    pub fn get(&self, path: &str) -> Option<&OscJsonNode> {
        let mut node = self;
        for part in path.split('/') {
            node = node.contents.as_ref()?.get(part)?;
        }
        Some(node)
    }


}

/// How a single logical shape maps to one or more OSC parameters
/// (a float channel, an optional sign bool, and/or bit-encoded bool channels).
#[derive(Clone)]
pub struct MysteryParam {
    pub main_address: Option<Arc<str>>,
    pub addresses: [Option<Arc<str>>; 7],
    pub neg_address: Option<Arc<str>>,
    pub num_bits: usize,
    pub last_value: f32,
    initialized: bool,
    pub last_bits: [bool; 8],
}

impl MysteryParam {
    pub fn invalidate(&mut self) { self.initialized = false; }
    pub fn new_float(address: &str) -> Self {
        Self {
            main_address: Some(address.into()),
            addresses: [None, None, None, None, None, None, None],
            neg_address: None,
            num_bits: 0,
            last_value: 0.0,
            initialized: false,
            last_bits: [false; 8],
        }
    }

    pub fn send(&mut self, value: f32, bundle: &mut OscBundle) {
        if let Some(addr) = self.main_address.as_ref() {
            if !self.initialized || (value - self.last_value).abs() > 0.01 {
                bundle.send_parameter(addr, OscType::Float(value));
                self.last_value = value;
            }
        }

        let mut value = value;
        if let Some(addr) = self.neg_address.as_ref() {
            let send_val = value < 0.;
            if !self.initialized || self.last_bits[7] != send_val {
                bundle.send_parameter(addr, OscType::Bool(send_val));
                self.last_bits[7] = send_val;
            }
            value = value.abs();
        } else if value < 0. {
            value = 0.;
        }

        let value = (value * ((1 << self.num_bits) - 1) as f32) as i32;

        self.addresses
            .iter()
            .enumerate()
            .take(self.num_bits)
            .for_each(|(idx, param)| {
                if let Some(addr) = param.as_ref() {
                    let send_val = value & (1 << idx) != 0;
                    if !self.initialized || self.last_bits[idx] != send_val {
                        bundle.send_parameter(addr, OscType::Bool(send_val));
                        self.last_bits[idx] = send_val;
                    }
                }
            });
        self.initialized = true;
    }
}

/// Parse a leaf parameter name into a base name plus an optional suffix
/// (`Negative` or a trailing power-of-two digit string), mirroring the
/// `^(.+?)(Negative|\d+)?$` regex used by oscavmgr.
pub(super) fn split_suffix(name: &str) -> (&str, Option<&str>) {
    if let Some(base) = name.strip_suffix("Negative") {
        return (base, Some("Negative"));
    }
    let digit_start = name
        .char_indices()
        .rev()
        .take_while(|(_, c)| c.is_ascii_digit())
        .last()
        .map(|(i, _)| i);
    if let Some(i) = digit_start {
        if i > 0 {
            return (&name[..i], Some(&name[i..]));
        }
    }
    (name, None)
}

/// Build the `[Option<MysteryParam>; NUM_SHAPES]` mapping from an OSC-JSON avatar
/// description. Leaves whose path contains `OSCm` are skipped.
pub fn build_params(node: &OscJsonNode) -> [Option<MysteryParam>; NUM_SHAPES] {
    let mut params: [Option<MysteryParam>; NUM_SHAPES] =
        std::array::from_fn(|_| None);

    let Some(parameters) = node.get("parameters") else {
        return params;
    };

    fn walk(node: &OscJsonNode, params: &mut [Option<MysteryParam>; NUM_SHAPES]) {
        if let Some(contents) = &node.contents {
            for child in contents.values() {
                walk(child, params);
            }
        } else {
            // Only writable float/bool avatar channels can receive shapes.
            if node.access & 2 == 0 || !matches!(node.data_type.as_deref(), Some("f" | "b")) {
                return;
            }
            let full = node.full_path.as_ref();
            if full.contains("OSCm") {
                return;
            }
            let name = full.rsplit('/').next().unwrap_or(full);
            if let Some(idx) = shape_index(name) {
                let next = make_param(name, node.full_path.clone());
                if let Some(param) = params[idx].as_mut() {
                    if next.main_address.is_some() { param.main_address = next.main_address; }
                    if next.neg_address.is_some() { param.neg_address = next.neg_address; }
                    for (target, source) in param.addresses.iter_mut().zip(next.addresses) {
                        if source.is_some() { *target = source; }
                    }
                    param.num_bits = param.num_bits.max(next.num_bits);
                } else {
                    params[idx] = Some(next);
                }
            }
        }
    }

    walk(parameters, &mut params);
    params
}

/// Resolve a leaf name to a shape index (Unified, then Combined).
pub(super) fn shape_index(name: &str) -> Option<usize> {
    let (base, _suffix) = split_suffix(name);
    // This oscavmgr source model supplies a shared vertical gaze value.
    if matches!(base, "EyeLeftY" | "EyeRightY") {
        return Some(UnifiedExpressions::EyeY as usize);
    }
    if let Ok(u) = UnifiedExpressions::from_str(base) {
        return Some(u as usize);
    }
    if let Ok(c) = CombinedExpression::from_str(base) {
        return Some(c as usize);
    }
    None
}

/// Construct a `MysteryParam` for a leaf, honoring `Negative` / digit suffixes.
fn make_param(name: &str, full_path: Arc<str>) -> MysteryParam {
    let (_, suffix) = split_suffix(name);
    let address = full_path
        .strip_prefix(super::bundle::PARAM_PREFIX)
        .unwrap_or(full_path.as_ref());

    let mut param = MysteryParam {
        main_address: None,
        addresses: [None, None, None, None, None, None, None],
        neg_address: None,
        num_bits: 0,
        last_value: 0.0,
        initialized: false,
        last_bits: [false; 8],
    };

    match suffix {
        None => param.main_address = Some(address.into()),
        Some("Negative") => param.neg_address = Some(address.into()),
        Some(digits) => {
            if let Ok(d) = digits.parse::<u32>() {
                if d.is_power_of_two() {
                    let idx = d.trailing_zeros() as usize;
                    if idx < 7 {
                        param.addresses[idx] = Some(address.into());
                        param.num_bits = param.num_bits.max(idx + 1);
                    }
                }
            }
        }
    }
    param
}

/// Default parameter mapping (used before an OSC-JSON avatar is loaded):
/// a small set of combined/unified shapes at `FT/v2/<Name>`.
pub fn default_params() -> [Option<MysteryParam>; NUM_SHAPES] {
    let mut params: [Option<MysteryParam>; NUM_SHAPES] = std::array::from_fn(|_| None);
    let defaults: &[(usize, &str)] = &[
        (CombinedExpression::BrowExpressionLeft as usize, "BrowExpressionLeft"),
        (CombinedExpression::BrowExpressionRight as usize, "BrowExpressionRight"),
        (CombinedExpression::EyeLidLeft as usize, "EyeLidLeft"),
        (CombinedExpression::EyeLidRight as usize, "EyeLidRight"),
        (CombinedExpression::JawX as usize, "JawX"),
        (CombinedExpression::LipFunnelLower as usize, "LipFunnelLower"),
        (CombinedExpression::LipFunnelUpper as usize, "LipFunnelUpper"),
        (CombinedExpression::LipPucker as usize, "LipPucker"),
        (CombinedExpression::MouthLowerDown as usize, "MouthLowerDown"),
        (CombinedExpression::MouthStretchTightenLeft as usize, "MouthStretchTightenLeft"),
        (CombinedExpression::MouthStretchTightenRight as usize, "MouthStretchTightenRight"),
        (CombinedExpression::MouthUpperUp as usize, "MouthUpperUp"),
        (CombinedExpression::MouthX as usize, "MouthX"),
        (CombinedExpression::SmileSadLeft as usize, "SmileSadLeft"),
        (CombinedExpression::SmileSadRight as usize, "SmileSadRight"),
        (UnifiedExpressions::CheekPuffLeft as usize, "CheekPuffLeft"),
        (UnifiedExpressions::CheekPuffRight as usize, "CheekPuffRight"),
        (UnifiedExpressions::EyeSquintLeft as usize, "EyeSquintLeft"),
        (UnifiedExpressions::EyeSquintRight as usize, "EyeSquintRight"),
        (UnifiedExpressions::EyeLeftX as usize, "EyeLeftX"),
        (UnifiedExpressions::EyeRightX as usize, "EyeRightX"),
        (UnifiedExpressions::EyeY as usize, "EyeY"),
        (UnifiedExpressions::JawOpen as usize, "JawOpen"),
        (UnifiedExpressions::MouthClosed as usize, "MouthClosed"),
    ];
    for (idx, name) in defaults {
        params[*idx] = Some(MysteryParam::new_float(&format!("FT/v2/{name}")));
    }
    params
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mapping_merges_float_sign_and_bit_channels_for_one_shape() {
        let node: OscJsonNode = serde_json::from_value(serde_json::json!({
            "FULL_PATH": "/avatar", "ACCESS": 0, "CONTENTS": {
                "parameters": { "FULL_PATH": "/avatar/parameters", "ACCESS": 0,
                    "CONTENTS": {
                        "JawX": {"FULL_PATH": "/avatar/parameters/Custom/JawX", "ACCESS": 2, "TYPE": "f"},
                        "JawXNegative": {"FULL_PATH": "/avatar/parameters/Custom/JawXNegative", "ACCESS": 2, "TYPE": "b"},
                        "JawX1": {"FULL_PATH": "/avatar/parameters/Custom/JawX1", "ACCESS": 2, "TYPE": "b"},
                        "JawX2": {"FULL_PATH": "/avatar/parameters/Custom/JawX2", "ACCESS": 2, "TYPE": "b"}
                    }
                }
            }
        })).unwrap();
        let mut params = build_params(&node);
        let param = params[CombinedExpression::JawX as usize].as_mut().unwrap();
        assert_eq!(param.main_address.as_deref(), Some("Custom/JawX"));
        assert_eq!(param.neg_address.as_deref(), Some("Custom/JawXNegative"));
        assert_eq!(param.num_bits, 2);
        let mut bundle = OscBundle::new_bundle();
        param.send(-1.0, &mut bundle);
        assert_eq!(bundle.content.len(), 4);
        assert!(bundle.content.iter().any(|packet| matches!(packet,
            rosc::OscPacket::Message(message) if message.addr.ends_with("Negative") && message.args == [rosc::OscType::Bool(true)])));
    }
}
