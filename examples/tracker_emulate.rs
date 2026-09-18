//! Emulate a single SlimeVR tracker sending a *known* rotation to the Java server,
//! so we can read back its bone output and validate the frame convention.
//!
//! ```text
//! cargo run --example tracker_emulate [seconds] [x y z w]
//! ```

use std::net::{Ipv4Addr, SocketAddr};
use std::time::Duration;

use firmware_protocol::deku::DekuContainerWrite as _;
use firmware_protocol::{
    BoardType, ImuType, McuType, Packet, SbPacket, SensorDataType, SlimeQuaternion, SlimeString,
};
use tokio::net::UdpSocket;

const SERVER_PORT: u16 = 6969;
const MAC: [u8; 6] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let seconds: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(5);
    // Default rotation: identity (w = 1).
    let q: [f32; 4] = std::env::args()
        .nth(2)
        .and_then(|_| {
            let a: Vec<f32> = std::env::args().skip(2).take(4).filter_map(|s| s.parse().ok()).collect();
            if a.len() == 4 { Some([a[0], a[1], a[2], a[3]]) } else { None }
        })
        .unwrap_or([0.0, 0.0, 0.0, 1.0]);

    let socket = UdpSocket::bind("0.0.0.0:0").await?;
    socket.set_broadcast(true)?;
    // Send directly to the local server (same host).
    let discovery: SocketAddr = SocketAddr::from((Ipv4Addr::LOCALHOST, SERVER_PORT));

    // 1. Broadcast handshake.
    let handshake = Packet::new(
        0u64,
        SbPacket::Handshake {
            board: BoardType::Haritora,
            imu: ImuType::Unknown(0),
            mcu: McuType::Haritora,
            imu_info: (0, 0, 0),
            build: 13,
            firmware: SlimeString::from("shora-parity"),
            mac_address: MAC,
        },
    )
    .to_bytes()
    .map_err(|e| anyhow::anyhow!("serialize handshake: {e:?}"))?;
    socket.send_to(&handshake, discovery).await?;

    // 2. Wait for the server's "Hey OVR =D 5" handshake response.
    let mut buf = [0u8; 2048];
    let mut server: Option<SocketAddr> = None;
    let mut deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    while server.is_none() && tokio::time::Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(500), socket.recv_from(&mut buf)).await {
            Ok(Ok((len, src))) => {
                eprintln!("recv {src} {len} bytes: {:?}", &buf[..len.min(20)]);
                if buf[..len].starts_with(b"\x03Hey OVR =D 5") {
                    server = Some(src);
                }
            }
            _ => {
                // keep broadcasting until we hear back
                socket.send_to(&handshake, discovery).await?;
            }
        }
    }
    let Some(server) = server else {
        eprintln!("no server handshake response");
        return Ok(());
    };
    eprintln!("connected to {server}");

    // 3. Send SENSOR_INFO (waist, position id 5 — safe, not used by the live
    //    shora trackers) — hand-rolled for the extra fields.
    let sensor_info = sensor_info_packet(1, 0, 5);
    socket.send_to(&sensor_info, server).await?;

    // 4. Stream the known rotation.
    let mut pn: u64 = 2;
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_secs(seconds) {
        let rot = Packet::new(
            pn,
            SbPacket::RotationData {
                sensor_id: 0,
                data_type: SensorDataType::Normal,
                quat: SlimeQuaternion { i: q[0], j: q[1], k: q[2], w: q[3] },
                calibration_info: 0,
            },
        )
        .to_bytes()
        .map_err(|e| anyhow::anyhow!("serialize rotation: {e:?}"))?;
        socket.send_to(&rot, server).await?;
        pn += 1;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    eprintln!("done");
    Ok(())
}

/// Hand-rolled SENSOR_INFO (type 15) with the v13 `tracker_position` field.
fn sensor_info_packet(pn: i64, sensor_id: u8, position: u8) -> Vec<u8> {
    let mut p = Vec::with_capacity(19);
    p.extend_from_slice(&15i32.to_be_bytes());
    p.extend_from_slice(&pn.to_be_bytes());
    p.push(sensor_id);
    p.push(1); // status OK
    p.push(0); // type unknown
    p.extend_from_slice(&[0, 3]); // sensorConfig: mag supported+enabled
    p.push(0); // hasCompletedRestCalibration
    p.push(position); // trackerPosition
    p
}
