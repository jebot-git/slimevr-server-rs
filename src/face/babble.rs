//! Project Babble + EyeTrackVR face/eye source, ported from oscavmgr's
//! `src/core/ext_tracking/babble.rs`.
//!
//! Project Babble (webcam mouth tracking) and EyeTrackVR (webcam/IR eye tracking)
//! are external programs that emit their results as OSC messages. This source
//! listens for those messages on a UDP port and maps their OSC addresses into the
//! unified shape model. Both programs can be used together on one port — Babble
//! uses bare `/…` addresses while EyeTrackVR uses `/avatar/parameters/…` addresses.

use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use anyhow::Context;
use rosc::{OscPacket, OscType};

use super::unified::{UnifiedExpressions, UnifiedTrackingData};
use super::FaceSource;

const FRESH: Duration = Duration::from_secs(1);

#[derive(Clone, Copy)]
struct Mapping {
    expr: UnifiedExpressions,
    invert: bool,
}

impl Mapping {
    const fn m(expr: UnifiedExpressions) -> Self {
        Self { expr, invert: false }
    }
    const fn inv(expr: UnifiedExpressions) -> Self {
        Self { expr, invert: true }
    }
}

/// Nonblocking input owned by the relay worker; no detached listener thread.
pub struct BabbleSource {
    socket: UdpSocket,
    last_received: Option<Instant>,
    last_eye_received: Option<Instant>,
}

impl BabbleSource {
    pub fn bind(addr: SocketAddr) -> anyhow::Result<Self> {
        let socket = UdpSocket::bind(addr)
            .with_context(|| format!("bind Babble/EyeTrackVR input at {addr}"))?;
        socket.set_nonblocking(true)?;
        tracing::info!(address = %socket.local_addr()?, "Babble/EyeTrackVR input listening");
        Ok(Self { socket, last_received: None, last_eye_received: None })
    }
}

impl FaceSource for BabbleSource {
    fn receive(&mut self, data: &mut UnifiedTrackingData) -> anyhow::Result<bool> {
        let mut buf = [0u8; 65535];
        // Bound work per tick so a busy sender cannot starve output or shutdown.
        for _ in 0..256 {
            match self.socket.recv_from(&mut buf) {
                Ok((size, _)) => {
                    if let Ok((_, packet)) = rosc::decoder::decode_udp(&buf[..size]) {
                        if apply_packet(&packet, data, &mut self.last_eye_received) {
                            self.last_received = Some(Instant::now());
                        }
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e).context("receive Babble/EyeTrackVR OSC"),
            }
        }
        data.eye_active = self.last_eye_received.is_some_and(|at| at.elapsed() < FRESH);
        Ok(self.last_received.is_some_and(|at| at.elapsed() < FRESH))
    }

    fn name(&self) -> &'static str { "babble" }
}

fn apply_packet(packet: &OscPacket, data: &mut UnifiedTrackingData, last_eye: &mut Option<Instant>) -> bool {
    match packet {
        OscPacket::Bundle(bundle) => {
            let mut changed = false;
            for packet in &bundle.content {
                changed |= apply_packet(packet, data, last_eye);
            }
            changed
        }
        OscPacket::Message(message) => {
            let Some(OscType::Float(value)) = message.args.first() else { return false };
            if !value.is_finite() { return false; }
            let mappings = mappings_for(&message.addr);
            for mapping in &mappings {
                data.setu(mapping.expr, if mapping.invert { 1.0 - value } else { *value });
                if mapping.expr as usize <= UnifiedExpressions::EyeWideLeft as usize {
                    *last_eye = Some(Instant::now());
                }
            }
            !mappings.is_empty()
        }
    }
}

/// Map an OSC address to the unified expression(s) it drives. Ported verbatim from
/// oscavmgr's `ADDR_TO_UNIFIED` table.
fn mappings_for(addr: &str) -> Vec<Mapping> {
    use UnifiedExpressions as U;

    match addr {
        // Project Babble — mouth
        "/cheekPuffLeft" => vec![Mapping::m(U::CheekPuffLeft)],
        "/cheekPuffRight" => vec![Mapping::m(U::CheekPuffRight)],
        "/cheekSuckLeft" => vec![Mapping::m(U::CheekSuckLeft)],
        "/cheekSuckRight" => vec![Mapping::m(U::CheekSuckRight)],
        "/jawOpen" => vec![Mapping::m(U::JawOpen)],
        "/jawForward" => vec![Mapping::m(U::JawForward)],
        "/jawLeft" => vec![Mapping::m(U::JawLeft)],
        "/jawRight" => vec![Mapping::m(U::JawRight)],
        "/noseSneerLeft" => vec![Mapping::m(U::NoseSneerLeft)],
        "/noseSneerRight" => vec![Mapping::m(U::NoseSneerRight)],
        "/mouthFunnel" => vec![
            Mapping::m(U::LipFunnelUpperRight),
            Mapping::m(U::LipFunnelUpperLeft),
            Mapping::m(U::LipFunnelLowerRight),
            Mapping::m(U::LipFunnelLowerLeft),
        ],
        "/mouthPucker" => vec![
            Mapping::m(U::LipPuckerUpperRight),
            Mapping::m(U::LipPuckerUpperLeft),
            Mapping::m(U::LipPuckerLowerRight),
            Mapping::m(U::LipPuckerLowerLeft),
        ],
        "/mouthLeft" => vec![Mapping::m(U::MouthPressLeft)],
        "/mouthRight" => vec![Mapping::m(U::MouthPressRight)],
        "/mouthRollUpper" => vec![Mapping::m(U::LipSuckUpperLeft), Mapping::m(U::LipSuckUpperRight)],
        "/mouthRollLower" => vec![Mapping::m(U::LipSuckLowerLeft), Mapping::m(U::LipSuckLowerRight)],
        "/mouthShrugUpper" => vec![Mapping::m(U::MouthRaiserUpper)],
        "/mouthShrugLower" => vec![Mapping::m(U::MouthRaiserLower)],
        "/mouthClose" => vec![Mapping::m(U::MouthClosed)],
        "/mouthSmileLeft" => vec![Mapping::m(U::MouthCornerPullLeft), Mapping::m(U::MouthCornerSlantLeft)],
        "/mouthSmileRight" => vec![Mapping::m(U::MouthCornerPullRight), Mapping::m(U::MouthCornerSlantRight)],
        "/mouthFrownLeft" => vec![Mapping::m(U::MouthFrownLeft), Mapping::m(U::MouthStretchLeft)],
        "/mouthFrownRight" => vec![Mapping::m(U::MouthFrownRight), Mapping::m(U::MouthStretchRight)],
        "/mouthDimpleLeft" => vec![Mapping::m(U::MouthDimpleLeft)],
        "/mouthDimpleRight" => vec![Mapping::m(U::MouthDimpleRight)],
        "/mouthUpperUpLeft" => vec![Mapping::m(U::MouthUpperUpLeft)],
        "/mouthUpperUpRight" => vec![Mapping::m(U::MouthUpperUpRight)],
        "/mouthLowerDownLeft" => vec![Mapping::m(U::MouthLowerDownLeft)],
        "/mouthLowerDownRight" => vec![Mapping::m(U::MouthLowerDownRight)],
        "/mouthStretchLeft" => vec![Mapping::m(U::MouthStretchLeft)],
        "/mouthStretchRight" => vec![Mapping::m(U::MouthStretchRight)],
        "/mouthPressLeft" => vec![Mapping::m(U::MouthPressLeft)],
        "/mouthPressRight" => vec![Mapping::m(U::MouthPressRight)],
        "/tongueOut" => vec![Mapping::m(U::TongueOut)],
        "/tongueUp" => vec![Mapping::m(U::TongueUp)],
        "/tongueDown" => vec![Mapping::m(U::TongueDown)],
        "/tongueLeft" => vec![Mapping::m(U::TongueLeft)],
        "/tongueRight" => vec![Mapping::m(U::TongueRight)],
        "/tongueRoll" => vec![Mapping::m(U::TongueRoll)],
        "/tongueBendDown" => vec![Mapping::m(U::TongueBendDown)],
        "/tongueCurlUp" => vec![Mapping::m(U::TongueCurlUp)],
        "/tongueSquish" => vec![Mapping::m(U::TongueSquish)],
        "/tongueFlat" => vec![Mapping::m(U::TongueFlat)],
        "/tongueTwistLeft" => vec![Mapping::m(U::TongueTwistLeft)],
        "/tongueTwistRight" => vec![Mapping::m(U::TongueTwistRight)],

        // Project Babble (Baballonia) — eye
        "/LeftEyeX" => vec![Mapping::m(U::EyeLeftX)],
        "/RightEyeX" => vec![Mapping::m(U::EyeRightX)],
        "/LeftEyeY" => vec![Mapping::m(U::EyeY)],
        // Babble's eye-lid is openness (1 = open) → invert to closedness.
        "/LeftEyeLid" => vec![Mapping::inv(U::EyeClosedLeft)],
        "/RightEyeLid" => vec![Mapping::inv(U::EyeClosedRight)],

        // EyeTrackVR
        "/avatar/parameters/LeftEyeX" => vec![Mapping::m(U::EyeLeftX)],
        "/avatar/parameters/RightEyeX" => vec![Mapping::m(U::EyeRightX)],
        "/avatar/parameters/EyesY" => vec![Mapping::m(U::EyeY)],
        "/avatar/parameters/LeftEyeLid" => vec![Mapping::m(U::EyeClosedLeft)],
        "/avatar/parameters/RightEyeLid" => vec![Mapping::m(U::EyeClosedRight)],
        "/avatar/parameters/v2/EyeLeftX" => vec![Mapping::m(U::EyeLeftX)],
        "/avatar/parameters/v2/EyeRightX" => vec![Mapping::m(U::EyeRightX)],
        "/avatar/parameters/v2/EyeLeftY" => vec![Mapping::m(U::EyeY)],
        "/avatar/parameters/v2/EyeLidLeft" => vec![Mapping::m(U::EyeClosedLeft)],
        "/avatar/parameters/v2/EyeLidRight" => vec![Mapping::m(U::EyeClosedRight)],

        _ => vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_babble_mouth() {
        let m = mappings_for("/cheekPuffLeft");
        assert_eq!(m.len(), 1);
        assert!(!m[0].invert);
        assert_eq!(m[0].expr as usize, UnifiedExpressions::CheekPuffLeft as usize);
    }

    #[test]
    fn maps_multi_output() {
        let m = mappings_for("/mouthFunnel");
        assert_eq!(m.len(), 4);
        assert_eq!(m[0].expr as usize, UnifiedExpressions::LipFunnelUpperRight as usize);
        assert_eq!(m[3].expr as usize, UnifiedExpressions::LipFunnelLowerLeft as usize);
    }

    #[test]
    fn maps_inverted_eye_lid() {
        let m = mappings_for("/LeftEyeLid");
        assert_eq!(m.len(), 1);
        assert!(m[0].invert);
        assert_eq!(m[0].expr as usize, UnifiedExpressions::EyeClosedLeft as usize);

        // ETVR's variant is not inverted.
        let m = mappings_for("/avatar/parameters/LeftEyeLid");
        assert!(!m[0].invert);
    }

    #[test]
    fn unknown_address_is_empty() {
        assert!(mappings_for("/does/not/exist").is_empty());
    }
}
