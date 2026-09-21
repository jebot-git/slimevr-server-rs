//! GX6/GX2 serial protocol decoder reused from Shora's haritorax-interpreter port.
use std::collections::{HashMap, HashSet};
use base64::Engine as _;
use super::{body_part_from_tracker_id, decode_imu, thigh_name_for_ankle, HaritoraXEvent};

const GX6_VID: u16 = 0x04DA;
const GX6_PID: u16 = 0x3F18;
const GX2_VID: u16 = 0x1915;
const GX2_PID: u16 = 0x520F;

#[derive(Default)]
pub struct Decoder {
    assignment: HashMap<char, String>,
    button_state: HashMap<String, (u8, u8)>,
    settings: HashMap<char, String>,
    lengths: HashMap<char, usize>,
    unknown: HashSet<char>,
    missing_assignments: HashSet<char>,
}

impl Decoder {
    pub fn line(&mut self, line: &str, is_x2: bool) -> Vec<HaritoraXEvent> {
        let mut events = Vec::new();
        process_line(line.trim(), is_x2, &mut self.assignment, &mut self.button_state,
            &mut self.settings, &mut self.lengths, &mut self.unknown, &mut self.missing_assignments, &mut events);
        events
    }

    /// Channels streaming samples before their identity report has arrived.
    /// Request these reports again instead of waiting for a tracker button/reset.
    pub fn missing_assignment_ports(&self) -> Vec<char> {
        let mut ports: Vec<_> = self.missing_assignments.iter().copied().collect();
        ports.sort_unstable();
        ports
    }

    pub fn disconnect(&self) -> Vec<HaritoraXEvent> {
        let mut names: HashSet<String> = self.assignment.values().cloned().collect();
        for name in self.assignment.values() {
            if let Some(thigh) = thigh_name_for_ankle(name) { names.insert(thigh.into()); }
        }
        names.into_iter().map(|name| HaritoraXEvent::TrackerDisconnected { name }).collect()
    }
}

/// Parse a single hex digit character into its value (0–15).
fn hex_char(c: Option<char>) -> Option<u8> {
    c?.to_digit(16).map(|d| d as u8)
}

/// Parse and dispatch a single dongle line.
#[allow(clippy::too_many_arguments)]
fn process_line(
    line: &str,
    is_x2: bool,
    assignment: &mut HashMap<char, String>,
    button_state: &mut HashMap<String, (u8, u8)>,
    settings_hex: &mut HashMap<char, String>,
    x_len_logged: &mut HashMap<char, usize>,
    logged_unknown: &mut HashSet<char>,
    missing_assignments: &mut HashSet<char>,
    events_tx: &mut Vec<HaritoraXEvent>,
) {
    let Some((head, payload)) = line.split_once(':') else {
        return;
    };
    // The dongle sends identifiers in mixed case (e.g. `X0:` for IMU data) — match
    // haritorax-interpreter by lowercasing the whole head before parsing.
    let head_lower = head.to_lowercase();
    let mut chars = head_lower.chars();
    let Some(identifier) = chars.next() else { return };
    let port_id: Option<char> = chars.next().filter(|c| c.is_ascii_digit());

    match identifier {
        'x' => {
            let Some(port_id) = port_id else {
                tracing::info!("GX6: x-frame without portId (head='{head}')");
                return;
            };
            let Some(name) = assignment.get(&port_id).cloned() else {
                if missing_assignments.insert(port_id) {
                    tracing::info!("GX6: x-frame for unassigned portId={port_id}");
                }
                return;
            };
            let Ok(bytes) = base64::engine::general_purpose::STANDARD.decode(payload) else {
                tracing::warn!("GX6: failed to base64-decode x-frame for {name}");
                return;
            };
            // Diagnostic: log when the frame size changes (first frame + any format change).
            if x_len_logged.get(&port_id) != Some(&payload.len()) {
                tracing::info!(
                    "GX6 x-frame: tracker={name} portId={port_id} base64_len={} decoded_len={}",
                    payload.len(),
                    bytes.len()
                );
                x_len_logged.insert(port_id, payload.len());
            }
            emit_imu(&name, &bytes, is_x2, events_tx);
        }
        'r' => {
            // Report: charAt(4) is the tracker id → body part; charAt(6)/charAt(9)
            // are the main/sub button counters (hex).
            let Some(port_id) = port_id else { return };
            let Some(tracker_id) = payload.chars().nth(4) else { return };
            let Some(name) = body_part_from_tracker_id(tracker_id) else { return };
            missing_assignments.remove(&port_id);

            if assignment.get(&port_id).map(String::as_str) != Some(name) {
                if let Some(old) = assignment.insert(port_id, name.to_string()) {
                    events_tx.push(HaritoraXEvent::TrackerDisconnected { name: old.clone() });
                    if let Some(thigh) = thigh_name_for_ankle(&old) {
                        events_tx.push(HaritoraXEvent::TrackerDisconnected { name: thigh.into() });
                    }
                }
                events_tx.push(HaritoraXEvent::TrackerConnected { name: name.into() });
            }

            if payload.len() >= 10 {
                let main = hex_char(payload.chars().nth(6));
                let sub = hex_char(payload.chars().nth(9));
                if let (Some(main), Some(sub)) = (main, sub) {
                    let prev = button_state.get(name).copied().unwrap_or((main, sub));
                    if main != 0 && main != prev.0 {
                        events_tx.push(HaritoraXEvent::Button {
                            name: name.to_string(),
                            button: "main".to_string(),
                        });
                    }
                    if sub != 0 && sub != prev.1 {
                        events_tx.push(HaritoraXEvent::Button {
                            name: name.to_string(),
                            button: "sub".to_string(),
                        });
                    }
                    button_state.insert(name.to_string(), (main, sub));
                }
            }
        }
        'v' => {
            if let Some(port_id) = port_id {
                if let Some(name) = assignment.get(&port_id).cloned() {
                    if let Ok(v) = serde_json::from_str::<serde_json::Value>(payload) {
                        let voltage_mv = v["battery voltage"].as_f64().unwrap_or(0.0);
                        let remaining = v["battery remaining"].as_f64().unwrap_or(0.0);
                        events_tx.push(HaritoraXEvent::Battery {
                            name,
                            voltage_v: (voltage_mv / 1000.0) as f32,
                            percentage: (remaining / 100.0) as f32,
                        });
                    }
                }
            }
        }
        'o' => {
            if let Some(port_id) = port_id {
                // Per-tracker settings echo (14-char hex) — used for power-off.
                if payload.len() == 14 && payload.bytes().all(|c| c.is_ascii_hexdigit()) {
                    settings_hex.insert(port_id, payload.to_string());
                }
            }
            // `o:` (no portId) is dongle-level channel info; not needed here.
        }
        'i' | 'a' => {
            // Info / RSSI heartbeats — no tracking data to forward.
        }
        other => {
            if logged_unknown.insert(other) {
                tracing::info!("GX6: unknown identifier '{other}' (head='{head}')");
            }
        }
    }
}

/// Emit one (or two, for HX2 legs) IMU event(s) for a decoded buffer.
fn emit_imu(
    name: &str,
    bytes: &[u8],
    is_x2: bool,
    events_tx: &mut Vec<HaritoraXEvent>,
) {
    if is_x2 && bytes.len() >= 30 && thigh_name_for_ankle(name).is_some() {
        // HaritoraX 2 legs: leg = [0..14], thigh = [16..30].
        if let Some(thigh) = thigh_name_for_ankle(name) {
            // The thigh "extension" has no `r:` report of its own — announce it here.
            events_tx.push(HaritoraXEvent::TrackerConnected {
                name: thigh.to_string(),
            });
        }
        if let Some(sample) = decode_imu(&bytes[0..14], is_x2) {
            events_tx.push(HaritoraXEvent::Imu {
                name: name.to_string(),
                sample,
            });
        }
        if bytes.len() >= 30 {
            if let Some(thigh) = thigh_name_for_ankle(name) {
                if let Some(sample) = decode_imu(&bytes[16..30], is_x2) {
                    events_tx.push(HaritoraXEvent::Imu {
                        name: thigh.to_string(),
                        sample,
                    });
                }
            }
        }
    } else if let Some(sample) = decode_imu(bytes, is_x2) {
        events_tx.push(HaritoraXEvent::Imu {
            name: name.to_string(),
            sample,
        });
    }
}

/// Discover all GX6/GX2 dongle serial ports by VID/PID. The GX6 presents up to
/// three USB serial ports, each carrying a subset of the paired trackers.
pub fn discover_dongle_ports() -> Vec<String> {
    let mut out = Vec::new();
    let Ok(ports) = serialport::available_ports() else {
        return out;
    };
    for p in ports {
        if let serialport::SerialPortType::UsbPort(info) = &p.port_type {
            if (info.vid == GX6_VID && info.pid == GX6_PID)
                || (info.vid == GX2_VID && info.pid == GX2_PID)
            {
                out.push(p.port_name.clone());
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(bytes: &[u8]) -> String {
        format!("X0:{}", base64::engine::general_purpose::STANDARD.encode(bytes))
    }

    #[test]
    fn x2_dual_leg_and_wireless_single_sensor() {
        let mut decoder = Decoder::default();
        decoder.line("R0:0000300000", true);
        let mut bytes = [0; 30];
        bytes[6..8].copy_from_slice(&(-18000_i16).to_le_bytes());
        bytes[16..18].copy_from_slice(&9000_i16.to_le_bytes());
        bytes[22..24].copy_from_slice(&(-15588_i16).to_le_bytes());
        let events = decoder.line(&frame(&bytes), true);
        let samples: Vec<_> = events.iter().filter_map(|e| match e {
            HaritoraXEvent::Imu { name, sample } => Some((name.as_str(), sample)), _ => None,
        }).collect();
        assert_eq!(samples.len(), 2);
        assert_eq!(samples[0].0, "leftAnkle");
        assert_eq!(samples[1].0, "leftKnee");
        assert!((samples[0].1.rotation[3] - 1.0).abs() < 1e-5);
        assert!((samples[1].1.rotation[0] - 0.5).abs() < 1e-4);
        assert_eq!(decoder.line(&frame(&bytes), false).iter()
            .filter(|e| matches!(e, HaritoraXEvent::Imu { .. })).count(), 1);
    }

    #[test]
    fn malformed_and_truncated_frames_do_not_panic() {
        let mut decoder = Decoder::default();
        decoder.line("r0:0000300000", true);
        for length in 0..40 {
            assert!(!decoder.line(&frame(&vec![0; length]), true).iter()
                .any(|e| matches!(e, HaritoraXEvent::Imu { .. })));
        }
        for line in ["", "X0:not-base64!", "r0:", "x:", "r0:💥", "v0:broken"] {
            assert!(decoder.line(line, true).is_empty());
        }
    }

    #[test]
    fn late_identity_reports_discover_both_knees_without_button_or_reset() {
        for (channel, tracker_id, ankle, knee) in [('0', '3', "leftAnkle", "leftKnee"),
            ('1', '5', "rightAnkle", "rightKnee")] {
            let mut decoder = Decoder::default();
            let mut bytes = [0u8; 30];
            bytes[6..8].copy_from_slice(&(-18000_i16).to_le_bytes());
            bytes[22..24].copy_from_slice(&(-18000_i16).to_le_bytes());
            let data = format!("X{channel}:{}", base64::engine::general_purpose::STANDARD.encode(bytes));
            assert!(decoder.line(&data, true).is_empty());
            assert_eq!(decoder.missing_assignment_ports(), vec![channel]);
            decoder.line(&format!("R{channel}:0000{tracker_id}00000"), true);
            assert!(decoder.missing_assignment_ports().is_empty());
            let events = decoder.line(&data, true);
            let names: Vec<_> = events.iter().filter_map(|event| match event {
                HaritoraXEvent::Imu { name, .. } => Some(name.as_str()), _ => None,
            }).collect();
            assert_eq!(names, vec![ankle, knee]);
            assert!(!events.iter().any(|event| matches!(event, HaritoraXEvent::Button { .. })));
        }
    }

    #[test]
    fn button_baseline_reassignment_and_disconnect() {
        let mut decoder = Decoder::default();
        let events = decoder.line("r0:0000305000", true);
        assert_eq!(events.len(), 1); // Initial counter must not trigger calibration.
        assert!(matches!(events[0], HaritoraXEvent::TrackerConnected { .. }));
        let events = decoder.line("r0:0000306000", true);
        assert!(matches!(&events[..], [HaritoraXEvent::Button { button, .. }] if button == "main"));
        assert!(decoder.line("r0:0000306000", true).is_empty());
        let events = decoder.line("r0:0000100000", true);
        assert!(matches!(&events[..], [
            HaritoraXEvent::TrackerDisconnected { name: ankle },
            HaritoraXEvent::TrackerDisconnected { name: thigh },
            HaritoraXEvent::TrackerConnected { name: chest },
        ] if ankle == "leftAnkle" && thigh == "leftKnee" && chest == "chest"));
        assert!(matches!(&decoder.disconnect()[..], [HaritoraXEvent::TrackerDisconnected { name }] if name == "chest"));
    }

    #[test]
    fn battery_units() {
        let mut decoder = Decoder::default();
        decoder.line("r0:0000600000", true);
        let events = decoder.line(r#"v0:{"battery voltage":3900,"battery remaining":75}"#, true);
        assert!(matches!(&events[..], [HaritoraXEvent::Battery { name, voltage_v, percentage }]
            if name == "hip" && (*voltage_v - 3.9).abs() < 1e-5 && *percentage == 0.75));
    }
}
