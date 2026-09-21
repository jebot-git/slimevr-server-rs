//! UniFT OSC output and VRChat avatar OSC JSON translation sheets.
//!
//! VRChat's `input` routes receive our output. Its `output` routes are feedback
//! from the avatar and must never be used as face-output destinations.
use std::collections::{HashMap, HashSet};
use anyhow::Context;
use rosc::{OscBundle, OscType};
use serde::Deserialize;
use strum::IntoEnumIterator;

use super::bundle::{AvatarBundle, PARAM_PREFIX};
use super::config::{FaceConfig, OutputMode};
use super::mystery::{build_params, default_params, shape_index, split_suffix, MysteryParam, OscJsonNode};
use super::unified::{CombinedExpression, UnifiedExpressions, UnifiedTrackingData, NUM_SHAPES};

pub enum FaceOutput {
    OscQuery(Box<[Option<MysteryParam>; NUM_SHAPES]>),
    Channels(ChannelMap),
}

impl FaceOutput {
    pub fn load(cfg: &FaceConfig) -> anyhow::Result<Self> {
        if let Some(path) = &cfg.translation_sheet {
            let text = std::fs::read_to_string(path)
                .with_context(|| format!("read face translation sheet {}", path.display()))?;
            let mut mapping = ChannelMap::from_avatar_json(&text)
                .with_context(|| format!("load face translation sheet {}", path.display()))?;
            if cfg.output == OutputMode::Both {
                // Explicit JSON routes take precedence where a canonical route overlaps.
                let explicit: HashSet<_> = mapping.channels.iter().map(|c| c.address.clone()).collect();
                mapping.channels.extend(ChannelMap::unift().channels.into_iter()
                    .filter(|c| !explicit.contains(&c.address)));
                mapping.has_eyelid = true;
            }
            return Ok(Self::Channels(mapping));
        }
        if let Some(path) = &cfg.avatar {
            let text = std::fs::read_to_string(path)
                .with_context(|| format!("read face avatar mapping {}", path.display()))?;
            let node: OscJsonNode = serde_json::from_str(&text).context("parse face avatar OSCQuery tree")?;
            anyhow::ensure!(node.get("parameters").is_some(),
                "face.avatar requires an OSCQuery /avatar tree; use face.translation_sheet for VRChat avatar JSON");
            return Ok(Self::OscQuery(Box::new(build_params(&node))));
        }
        if cfg.output == OutputMode::Unift {
            Ok(Self::Channels(ChannelMap::unift()))
        } else {
            // Preserve oscavmgr's fallback and discovery behavior in automatic mode.
            Ok(Self::OscQuery(Box::new(default_params())))
        }
    }

    pub fn discovers_avatar(cfg: &FaceConfig) -> bool {
        cfg.output == OutputMode::Auto && cfg.osc_query && cfg.avatar.is_none()
            && cfg.translation_sheet.is_none()
    }

    pub fn from_oscquery(node: &OscJsonNode) -> Self {
        Self::OscQuery(Box::new(build_params(node)))
    }

    pub fn invalidate(&mut self, data: &mut UnifiedTrackingData) {
        data.invalidate_output();
        match self {
            Self::OscQuery(params) => {
                for param in params.iter_mut().flatten() { param.invalidate(); }
            }
            Self::Channels(mapping) => {
                for channel in &mut mapping.channels { channel.previous = None; }
            }
        }
    }

    pub fn apply(&mut self, data: &mut UnifiedTrackingData, bundle: &mut OscBundle, active: bool) {
        match self {
            Self::OscQuery(params) => data.apply_to_bundle(params, bundle, active),
            Self::Channels(mapping) => {
                mapping.send(data, active, bundle);
                data.apply_eye_tracking(bundle, mapping.has_eyelid);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
enum ParameterType {
    #[serde(alias = "float")]
    Float,
    #[serde(alias = "bool")]
    Bool,
    #[serde(alias = "int")]
    Int,
}

#[derive(Clone, Copy, Debug)]
enum ValueSource { Shape(usize), TrackingActive, EyeTrackingActive }
#[derive(Clone, Copy, Debug)]
enum Encoding { Direct, Negative, Bit { index: u32, bits: u32, signed: bool } }

struct Channel {
    source: ValueSource,
    encoding: Encoding,
    address: String,
    kind: ParameterType,
    previous: Option<OscType>,
}
impl Channel {
    fn send(&mut self, data: &UnifiedTrackingData, active: bool, bundle: &mut OscBundle) {
        let raw = match self.source {
            ValueSource::Shape(index) => data.shapes[index],
            ValueSource::TrackingActive => if active { 1.0 } else { 0.0 },
            ValueSource::EyeTrackingActive => if active && data.eye_active { 1.0 } else { 0.0 },
        };
        if !raw.is_finite() { return; }
        let value = match self.encoding {
            Encoding::Direct => raw,
            Encoding::Negative => if raw < 0.0 { 1.0 } else { 0.0 },
            Encoding::Bit { index, bits, signed } => {
                let magnitude = if signed { raw.abs() } else { raw };
                let quantized = (magnitude.clamp(0.0, 1.0) * ((1_u32 << bits) - 1) as f32) as u32;
                if quantized & (1 << index) != 0 { 1.0 } else { 0.0 }
            }
        };
        let value = match self.kind {
            ParameterType::Float => OscType::Float(value),
            ParameterType::Bool => OscType::Bool(value > 0.5),
            ParameterType::Int => OscType::Int(value as i32),
        };
        let changed = match (&self.previous, &value) {
            (Some(OscType::Float(old)), OscType::Float(new)) => (old - new).abs() > 0.01,
            (Some(old), new) => old != new,
            (None, _) => true,
        };
        if changed {
            bundle.send_tracking(&self.address, vec![value.clone()]);
            self.previous = Some(value);
        }
    }
}

/// Compiled routes. Multiple avatar parameters can consume one logical shape.
pub struct ChannelMap {
    channels: Vec<Channel>,
    has_eyelid: bool,
}
impl ChannelMap {
    fn unift() -> Self {
        let names = UnifiedExpressions::iter().map(|expr| (expr as usize, <&'static str>::from(expr)))
            .chain(CombinedExpression::iter().map(|expr| (expr as usize, <&'static str>::from(expr))));
        let mut channels: Vec<_> = names.map(|(index, name)| Channel {
            source: ValueSource::Shape(index), encoding: Encoding::Direct,
            address: format!("{PARAM_PREFIX}FT/v2/{name}"), kind: ParameterType::Float, previous: None,
        }).collect();
        for name in ["EyeLeftY", "EyeRightY"] {
            channels.push(Channel { source: ValueSource::Shape(UnifiedExpressions::EyeY as usize),
                encoding: Encoding::Direct, address: format!("{PARAM_PREFIX}FT/v2/{name}"),
                kind: ParameterType::Float, previous: None });
        }
        for name in ["ExpressionTrackingActive", "LipTrackingActive", "EyeTrackingActive"] {
            let source = if name == "EyeTrackingActive" { ValueSource::EyeTrackingActive } else { ValueSource::TrackingActive };
            channels.push(Channel { source, encoding: Encoding::Direct,
                address: format!("{PARAM_PREFIX}{name}"), kind: ParameterType::Bool, previous: None });
        }
        Self { channels, has_eyelid: true }
    }

    fn from_avatar_json(text: &str) -> anyhow::Result<Self> {
        #[derive(Deserialize)]
        struct Avatar { parameters: Vec<Parameter> }
        #[derive(Deserialize)]
        struct Parameter { name: String, input: Option<Input> }
        #[derive(Deserialize)]
        struct Input { address: String, #[serde(rename = "type")] kind: ParameterType }
        let avatar: Avatar = serde_json::from_str(text).context("parse VRChat avatar OSC JSON")?;
        anyhow::ensure!(avatar.parameters.len() <= 8192, "too many avatar parameters (maximum 8192)");
        let mut groups: HashMap<String, (u32, bool)> = HashMap::new();
        let mut channels = Vec::new();
        let mut group_names = Vec::new();
        let mut addresses = HashSet::new();
        let mut has_eyelid = false;
        for parameter in avatar.parameters {
            let Some(input) = parameter.input else { continue; };
            // Resolve by avatar parameter NAME, never the remapped OSC address.
            let name = parameter.name.rsplit('/').next().unwrap_or(&parameter.name);
            let (base, suffix) = split_suffix(name);
            let source = if name == "EyeTrackingActive" {
                ValueSource::EyeTrackingActive
            } else if matches!(name, "ExpressionTrackingActive" | "LipTrackingActive") {
                ValueSource::TrackingActive
            } else if let Some(index) = shape_index(name) {
                has_eyelid |= index == CombinedExpression::EyeLidLeft as usize;
                ValueSource::Shape(index)
            } else {
                continue; // Unrelated avatar controls such as VRCEmote are not face routes.
            };
            anyhow::ensure!(input.address.starts_with('/') && input.address.len() > 1
                && !input.address.chars().any(|c| c.is_whitespace() || c.is_control()
                    || matches!(c, '*' | '?' | '[' | ']' | '{' | '}' | ',' | '#')),
                "invalid OSC input address {:?} for {:?}", input.address, parameter.name);
            anyhow::ensure!(addresses.insert(input.address.clone()),
                "duplicate OSC input address {:?} in face mappings", input.address);
            let group = match parameter.name.rsplit_once('/') {
                Some((prefix, _)) => format!("{prefix}/{base}"), None => base.into(),
            };
            let format = groups.entry(group.clone()).or_default();
            let encoding = match suffix {
                None => Encoding::Direct,
                Some("Negative") => { format.1 = true; Encoding::Negative }
                Some(digit) => {
                    let power: u32 = digit.parse().context("invalid binary face parameter suffix")?;
                    anyhow::ensure!(power.is_power_of_two() && power <= 64,
                        "binary face parameter {:?} must end in 1, 2, 4, 8, 16, 32 or 64", parameter.name);
                    let index = power.trailing_zeros();
                    format.0 = format.0.max(index + 1);
                    Encoding::Bit { index, bits: 0, signed: false }
                }
            };
            group_names.push(group);
            channels.push(Channel { source, encoding, address: input.address, kind: input.kind, previous: None });
        }
        anyhow::ensure!(!channels.is_empty(),
            "avatar JSON contains no supported face input mappings; names must use Unified Expressions (e.g. FT/v2/JawOpen)");
        for (channel, group) in channels.iter_mut().zip(group_names) {
            if let Encoding::Bit { bits, signed, .. } = &mut channel.encoding {
                (*bits, *signed) = groups[&group];
            }
        }
        Ok(Self { channels, has_eyelid })
    }

    fn send(&mut self, data: &UnifiedTrackingData, active: bool, bundle: &mut OscBundle) {
        for channel in &mut self.channels { channel.send(data, active, bundle); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rosc::OscPacket;
    use serde_json::json;

    fn values(bundle: OscBundle) -> HashMap<String, OscType> {
        bundle.content.into_iter().map(|packet| match packet {
            OscPacket::Message(msg) => (msg.addr, msg.args[0].clone()),
            _ => panic!("expected messages"),
        }).collect()
    }

    #[test]
    fn unift_emits_all_supported_shapes_and_vertical_gaze_aliases() {
        let mut map = ChannelMap::unift();
        let mut data = UnifiedTrackingData::default();
        data.setu(UnifiedExpressions::TongueOut, 0.7);
        data.setu(UnifiedExpressions::EyeY, -0.3);
        data.setc(CombinedExpression::JawX, -0.6);
        let mut bundle = OscBundle::new_bundle();
        map.send(&data, true, &mut bundle);
        let values = values(bundle);
        assert_eq!(values.len(), NUM_SHAPES + 5);
        assert_eq!(values["/avatar/parameters/FT/v2/TongueOut"], OscType::Float(0.7));
        assert_eq!(values["/avatar/parameters/FT/v2/JawX"], OscType::Float(-0.6));
        for name in ["EyeY", "EyeLeftY", "EyeRightY"] {
            assert_eq!(values[&format!("{PARAM_PREFIX}FT/v2/{name}")], OscType::Float(-0.3));
        }
        assert_eq!(values["/avatar/parameters/ExpressionTrackingActive"], OscType::Bool(true));
    }

    #[test]
    fn vrchat_name_drives_input_route_and_output_only_routes_are_ignored() {
        let mut map = ChannelMap::from_avatar_json(&json!({"parameters": [
            {"name": "FT/v2/JawOpen", "input": {"address": "/custom/jaw", "type": "Float"},
                "output": {"address": "/wrong/direction", "type": "Float"}},
            {"name": "Other/v2/JawOpen", "input": {"address": "/custom/mirror", "type": "float"}},
            {"name": "FT/v2/TongueOut", "output": {"address": "/feedback/tongue", "type": "Float"}},
            {"name": "VRCEmote", "input": {"address": "/avatar/parameters/VRCEmote", "type": "Int"}},
            {"name": "ExpressionTrackingActive", "input": {"address": "/custom/active", "type": "Bool"}}
        ]}).to_string()).unwrap();
        let mut data = UnifiedTrackingData::default();
        data.setu(UnifiedExpressions::JawOpen, 0.7);
        let mut bundle = OscBundle::new_bundle();
        map.send(&data, true, &mut bundle);
        let found = values(bundle);
        assert_eq!(found.len(), 3);
        assert_eq!(found["/custom/jaw"], OscType::Float(0.7));
        assert_eq!(found["/custom/mirror"], OscType::Float(0.7));
        assert_eq!(found["/custom/active"], OscType::Bool(true));
    }

    #[test]
    fn json_types_binary_sign_and_per_prefix_resolution() {
        let mut map = ChannelMap::from_avatar_json(&json!({"parameters": [
            {"name": "FT/v2/JawX", "input": {"address": "/jaw/float", "type": "Float"}},
            {"name": "FT/v2/JawXNegative", "input": {"address": "/jaw/sign", "type": "Bool"}},
            {"name": "FT/v2/JawX1", "input": {"address": "/jaw/bit0", "type": "Bool"}},
            {"name": "FT/v2/JawX2", "input": {"address": "/jaw/bit1", "type": "Bool"}},
            {"name": "Other/v2/JawX1", "input": {"address": "/unsigned/bit0", "type": "Bool"}},
            {"name": "Bool/v2/JawOpen", "input": {"address": "/jaw/bool", "type": "Bool"}},
            {"name": "Int/v2/JawOpen", "input": {"address": "/jaw/int", "type": "Int"}}
        ]}).to_string()).unwrap();
        let mut data = UnifiedTrackingData::default();
        data.setc(CombinedExpression::JawX, -0.8);
        data.setu(UnifiedExpressions::JawOpen, 1.0);
        let mut bundle = OscBundle::new_bundle();
        map.send(&data, true, &mut bundle);
        let found = values(bundle);
        assert_eq!(found["/jaw/float"], OscType::Float(-0.8));
        assert_eq!(found["/jaw/sign"], OscType::Bool(true));
        assert_eq!(found["/jaw/bit0"], OscType::Bool(false));
        assert_eq!(found["/jaw/bit1"], OscType::Bool(true));
        assert_eq!(found["/unsigned/bit0"], OscType::Bool(false));
        assert_eq!(found["/jaw/bool"], OscType::Bool(true));
        assert_eq!(found["/jaw/int"], OscType::Int(1));
    }

    #[test]
    fn invalid_sheets_fail_instead_of_silently_using_defaults() {
        for text in ["{}", "{broken", r#"{"parameters":[]}"#,
            r#"{"parameters":[{"name":"VRCEmote","input":{"address":"/emote","type":"Int"}}]}"#,
            r#"{"parameters":[{"name":"JawOpen","input":{"address":"/jaw","type":"String"}}]}"#,
            r#"{"parameters":[{"name":"JawOpen","input":{"address":"jaw","type":"Float"}}]}"#,
            r#"{"parameters":[{"name":"JawX3","input":{"address":"/jaw","type":"Bool"}}]}"#,
            r#"{"parameters":[{"name":"JawX128","input":{"address":"/jaw","type":"Bool"}}]}"#,
            r#"{"parameters":[{"name":"JawOpen","input":{"address":"/same","type":"Float"}},{"name":"TongueOut","input":{"address":"/same","type":"Float"}}]}"#,
        ] {
            assert!(ChannelMap::from_avatar_json(text).is_err(), "accepted {text}");
        }
        ChannelMap::from_avatar_json(include_str!("../../examples/face/avatar-osc.json")).unwrap();
    }

    #[test]
    fn avatar_change_resends_unchanged_values_and_static_modes_disable_discovery() {
        let mut output = FaceOutput::Channels(ChannelMap::unift());
        let mut data = UnifiedTrackingData::default();
        let mut first = OscBundle::new_bundle();
        output.apply(&mut data, &mut first, true);
        let mut repeated = OscBundle::new_bundle();
        output.apply(&mut data, &mut repeated, true);
        assert!(repeated.content.is_empty());
        output.invalidate(&mut data);
        output.apply(&mut data, &mut repeated, true);
        assert_eq!(first.content, repeated.content);
        let mut cfg = FaceConfig::default();
        assert!(FaceOutput::discovers_avatar(&cfg));
        cfg.translation_sheet = Some("avatar.json".into());
        assert!(!FaceOutput::discovers_avatar(&cfg));
        cfg.translation_sheet = None;
        cfg.output = OutputMode::Unift;
        assert!(!FaceOutput::discovers_avatar(&cfg));
    }
}
