//! Sniff the live SlimeVR tracker UDP input on :6969 (co-listening alongside the
//! Java server via SO_REUSEADDR/SO_REUSEPORT) and print each tracker's rotation.
//!
//! ```text
//! cargo run --example tracker_sniff
//! ```

use std::net::SocketAddr;

use firmware_protocol::deku::DekuContainerRead as _;
use firmware_protocol::{Packet, SbPacket};
use socket2::{Domain, Protocol, Socket, Type};
use tokio::net::UdpSocket;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let addr: SocketAddr = "0.0.0.0:6969".parse()?;

    let sock = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    sock.set_reuse_address(true)?;
    #[cfg(unix)]
    sock.set_reuse_port(true)?;
    sock.set_nonblocking(true)?;
    sock.bind(&addr.into())?;
    let socket = UdpSocket::from_std(sock.into())?;

    eprintln!("sniffing {addr} (SO_REUSEADDR)");

    let mut buf = [0u8; 2048];
    loop {
        let Ok((len, src)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        let data = &buf[..len];
        if let Ok(((_, _), packet)) = Packet::<SbPacket>::from_bytes((data, 0)) {
            let (_, sb) = packet.split();
            if let SbPacket::RotationData {
                sensor_id, quat, ..
            } = sb
            {
                println!(
                    "rot {src} id={sensor_id} q=({:.4},{:.4},{:.4},{:.4})",
                    quat.i, quat.j, quat.k, quat.w
                );
            }
        }
    }
}
