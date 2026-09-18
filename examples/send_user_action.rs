//! Send a tracker user action (reset) to the server.
//!
//! ```text
//! cargo run --example send_user_action [reset|yaw|mounting|pause]
//! ```

use std::net::SocketAddr;

use firmware_protocol::deku::DekuContainerWrite as _;
use firmware_protocol::{ActionType, Packet, SbPacket};
use tokio::net::UdpSocket;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let action = match std::env::args().nth(1).as_deref() {
        Some("yaw") => ActionType::ResetYaw,
        Some("mounting") => ActionType::ResetMounting,
        Some("pause") => ActionType::PauseTracking,
        _ => ActionType::Reset,
    };

    let socket = UdpSocket::bind("0.0.0.0:0").await?;
    let action_name = format!("{action:?}");
    let pkt = Packet::new(0, SbPacket::UserAction { action })
        .to_bytes()
        .map_err(|e| anyhow::anyhow!("serialize user action: {e:?}"))?;
    let dst: SocketAddr = "127.0.0.1:6969".parse()?;
    socket.send_to(&pkt, dst).await?;
    eprintln!("sent user action {action_name} to {dst}");
    Ok(())
}
