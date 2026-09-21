use super::*;
use rosc::{OscMessage, OscType};
use std::collections::HashMap;

fn test_config() -> (FaceConfig, UdpSocket) {
    let sink = UdpSocket::bind("127.0.0.1:0").unwrap();
    sink.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
    let feedback = UdpSocket::bind("127.0.0.1:0").unwrap();
    let babble = UdpSocket::bind("127.0.0.1:0").unwrap();
    let cfg = FaceConfig {
        enabled: true,
        osc_query: false,
        destination: sink.local_addr().unwrap(),
        osc_port: feedback.local_addr().unwrap().port(),
        babble_port: babble.local_addr().unwrap().port(),
        ..FaceConfig::default()
    };
    (cfg, sink)
}

fn message(addr: &str, value: OscType) -> OscPacket {
    OscPacket::Message(OscMessage { addr: addr.into(), args: vec![value] })
}

fn collect(packet: OscPacket, values: &mut HashMap<String, OscType>) {
    match packet {
        OscPacket::Message(message) => {
            if let Some(value) = message.args.into_iter().next() { values.insert(message.addr, value); }
        }
        OscPacket::Bundle(bundle) => {
            for packet in bundle.content { collect(packet, values); }
        }
    }
}

fn wait_values(sink: &UdpSocket, expected: &[(&str, OscType)]) -> HashMap<String, OscType> {
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut values = HashMap::new();
    let mut buf = [0; 65535];
    while Instant::now() < deadline {
        match sink.recv_from(&mut buf) {
            Ok((size, _)) => collect(rosc::decoder::decode_udp(&buf[..size]).unwrap().1, &mut values),
            Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {}
            Err(e) => panic!("receive output: {e}"),
        }
        if expected.iter().all(|(key, value)| values.get(*key).is_some_and(|found| {
            match (found, value) {
                (OscType::Float(a), OscType::Float(b)) => (a - b).abs() < 1e-5,
                _ => found == value,
            }
        })) { return values; }
    }
    panic!("missing expected OSC values {expected:?}; received {values:?}");
}

#[test]
fn babble_bundles_reach_output_expire_and_release_ports() {
    let (mut cfg, sink) = test_config();
    cfg.expose = true; // Input exposure must never turn output into 0.0.0.0.
    let mut relay = FaceRelay::start(cfg.clone()).unwrap();
    let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
    let packet = OscPacket::Bundle(OscBundle {
        timetag: (0, 1).into(),
        content: vec![
            message("/jawOpen", OscType::Float(0.65)),
            OscPacket::Bundle(OscBundle {
                timetag: (0, 1).into(),
                content: vec![message("/LeftEyeLid", OscType::Float(0.25)),
                    message("/avatar/parameters/RightEyeLid", OscType::Float(0.2)),
                    message("/avatar/parameters/LeftEyeX", OscType::Float(-0.3))],
            }),
        ],
    });
    sender.send_to(b"not OSC", ("127.0.0.1", cfg.babble_port)).unwrap();
    sender.send_to(&rosc::encoder::encode(&packet).unwrap(), ("127.0.0.1", cfg.babble_port)).unwrap();
    wait_values(&sink, &[
        ("/avatar/parameters/FT/v2/JawOpen", OscType::Float(0.65)),
        ("/avatar/parameters/FT/v2/EyeLeftX", OscType::Float(-0.3)),
        ("/avatar/parameters/FT/v2/EyeLidLeft", OscType::Float(0.0)),
        ("/avatar/parameters/FT/v2/EyeLidRight", OscType::Float(0.525)),
        ("/avatar/parameters/ExpressionTrackingActive", OscType::Bool(true)),
    ]);
    wait_values(&sink, &[
        ("/avatar/parameters/ExpressionTrackingActive", OscType::Bool(false)),
        ("/avatar/parameters/FT/v2/JawOpen", OscType::Float(0.0)),
    ]);
    relay.check_health().unwrap();
    drop(relay);
    let _feedback = UdpSocket::bind(("0.0.0.0", cfg.osc_port)).unwrap();
    let _babble = UdpSocket::bind(("0.0.0.0", cfg.babble_port)).unwrap();
}

#[test]
fn occupied_source_port_fails_startup_and_releases_feedback() {
    let (cfg, _sink) = test_config();
    let _occupied = UdpSocket::bind(("127.0.0.1", cfg.babble_port)).unwrap();
    let error = match FaceRelay::start(cfg.clone()) {
        Ok(_) => panic!("startup should fail"),
        Err(error) => error,
    };
    assert!(format!("{error:#}").contains("Babble/EyeTrackVR"));
    let _feedback = UdpSocket::bind(("127.0.0.1", cfg.osc_port)).unwrap();
}

#[test]
fn avatar_mapping_errors_are_reported_before_startup() {
    let cfg = FaceConfig {
        avatar: Some(std::path::PathBuf::from("/nonexistent/slimevr-avatar.json")),
        ..FaceConfig::default()
    };
    assert!(FaceOutput::load(&cfg).is_err());
}

#[test]
fn bundled_feedback_updates_cache_and_vsync() {
    let mut cache = FaceParams::new();
    let mut vsync = false;
    let packet = OscPacket::Bundle(OscBundle {
        timetag: (0, 1).into(),
        content: vec![message("/avatar/parameters/VSync", OscType::Bool(true)),
            message("/avatar/parameters/Test", OscType::Float(0.5))],
    });
    handle_feedback(&packet, &mut cache, &mut vsync, &mut None);
    assert!(vsync);
    assert_eq!(cache["Test"], OscType::Float(0.5));
    handle_feedback(&message("/avatar/change", OscType::String("avtr_test".into())),
        &mut cache, &mut vsync, &mut None);
    assert!(cache.is_empty());
}

#[test]
fn unift_and_vrchat_json_outputs_reach_udp_and_clear_on_expiry_and_shutdown() {
    use super::super::config::OutputMode;
    for mode in [OutputMode::Unift, OutputMode::Json, OutputMode::Both] {
        let (mut cfg, sink) = test_config();
        cfg.output = mode;
        if mode != OutputMode::Unift {
            cfg.translation_sheet = Some(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/face/avatar-osc.json").into());
        }
        let relay = FaceRelay::start(cfg.clone()).unwrap();
        let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
        let bytes = rosc::encoder::encode(&OscPacket::Bundle(OscBundle {
            timetag: (0, 1).into(), content: vec![
                message("/jawOpen", OscType::Float(0.65)),
                message("/jawLeft", OscType::Float(1.0)),
                message("/tongueOut", OscType::Float(0.8)),
                message("/LeftEyeX", OscType::Float(-0.3)),
            ],
        })).unwrap();
        sender.send_to(&bytes, ("127.0.0.1", cfg.babble_port)).unwrap();
        let mut expected = vec![];
        if mode != OutputMode::Json {
            expected.extend([
                ("/avatar/parameters/FT/v2/JawOpen", OscType::Float(0.65)),
                ("/avatar/parameters/FT/v2/TongueOut", OscType::Float(0.8)),
                ("/avatar/parameters/FT/v2/JawX", OscType::Float(-1.0)),
                ("/avatar/parameters/ExpressionTrackingActive", OscType::Bool(true)),
                ("/avatar/parameters/EyeTrackingActive", OscType::Bool(true)),
            ]);
        }
        if mode != OutputMode::Unift {
            expected.extend([
                ("/avatar/parameters/CustomMouthOpen", OscType::Float(0.65)),
                ("/avatar/parameters/JawSign", OscType::Bool(true)),
                ("/avatar/parameters/JawBit0", OscType::Bool(true)),
                ("/avatar/parameters/JawBit1", OscType::Bool(true)),
                ("/avatar/parameters/CustomTrackingActive", OscType::Bool(true)),
                ("/avatar/parameters/CustomEyeTrackingActive", OscType::Bool(true)),
            ]);
        }
        let received = wait_values(&sink, &expected);
        assert!(!received.contains_key("/feedback/mouth"));
        if mode == OutputMode::Json {
            assert!(!received.keys().any(|key| key.starts_with("/avatar/parameters/FT/v2/")));
        }
        // Static sheets must remain in use, and unchanged values must be sent to
        // the new avatar even though change suppression already cached them.
        sender.send_to(&rosc::encoder::encode(&message("/avatar/change", OscType::String("avtr_next".into()))).unwrap(),
            ("127.0.0.1", cfg.osc_port)).unwrap();
        wait_values(&sink, &expected);
        let (jaw, tracking) = if mode == OutputMode::Unift {
            ("/avatar/parameters/FT/v2/JawOpen", "/avatar/parameters/ExpressionTrackingActive")
        } else {
            ("/avatar/parameters/CustomMouthOpen", "/avatar/parameters/CustomTrackingActive")
        };
        wait_values(&sink, &[(jaw, OscType::Float(0.0)), (tracking, OscType::Bool(false))]);
        sender.send_to(&bytes, ("127.0.0.1", cfg.babble_port)).unwrap();
        wait_values(&sink, &[(jaw, OscType::Float(0.65)), (tracking, OscType::Bool(true))]);
        drop(relay);
        wait_values(&sink, &[(jaw, OscType::Float(0.0)), (tracking, OscType::Bool(false))]);
    }
}

#[test]
fn invalid_json_mapping_fails_before_binding_ports() {
    let (mut cfg, _sink) = test_config();
    cfg.output = super::super::config::OutputMode::Json;
    cfg.translation_sheet = Some("/nonexistent/avatar-translation-sheet.json".into());
    assert!(FaceRelay::start(cfg.clone()).is_err());
    let _feedback = UdpSocket::bind(("127.0.0.1", cfg.osc_port)).unwrap();
    let _babble = UdpSocket::bind(("127.0.0.1", cfg.babble_port)).unwrap();
}
