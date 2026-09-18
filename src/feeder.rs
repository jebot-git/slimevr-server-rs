//! SteamVR feeder bridge — the `SlimeVRInput` Unix socket where WiVRn sends the
//! HMD (and controller) 6-DoF poses. Framing matches the SolarXR socket: a
//! 4-byte little-endian length prefix (including the 4 prefix bytes) followed by
//! a protobuf `messages.ProtobufMessage`.

use std::io;
use std::path::Path;
use std::sync::{Arc, RwLock};

use prost::Message;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite};
use tokio::net::{UnixListener, UnixStream};

mod proto {
    include!(concat!(env!("OUT_DIR"), "/messages.rs"));
}
use proto::{protobuf_message, ProtobufMessage};

/// The HMD pose reported by WiVRn: `[x, y, z, qx, qy, qz, qw]`.
pub type HmdPose = [f32; 7];

/// SlimeVR tracker role for the HMD (see the `TrackerRole` enum).
const ROLE_HMD: i32 = 19;

/// Run the feeder bridge forever, recording the latest HMD pose in `hmd`.
pub async fn run(path: String, hmd: Arc<RwLock<Option<HmdPose>>>) -> anyhow::Result<()> {
    let p = Path::new(&path);
    if p.exists() {
        if UnixStream::connect(p).await.is_ok() {
            anyhow::bail!("feeder socket {path:?} is already in use");
        }
        tracing::warn!("removing stale feeder socket {path:?}");
        std::fs::remove_file(p)?;
    }
    let listener = UnixListener::bind(p)?;
    tracing::info!("SteamVR feeder socket listening on {path}");

    loop {
        let (stream, _) = listener.accept().await?;
        tracing::info!("feeder client connected");
        let hmd = hmd.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(stream, hmd).await {
                tracing::warn!("feeder connection ended: {e:#}");
            }
        });
    }
}

/// Serve one WiVRn feeder connection: learn the HMD tracker id, record its pose.
async fn handle_connection(
    mut stream: UnixStream,
    hmd: Arc<RwLock<Option<HmdPose>>>,
) -> anyhow::Result<()> {
    let mut hmd_id: Option<i32> = None;
    loop {
        let Some(body) = read_message(&mut stream).await? else {
            break;
        };
        let Ok(msg) = ProtobufMessage::decode(body.as_slice()) else {
            continue;
        };
        match msg.message {
            Some(protobuf_message::Message::TrackerAdded(ta)) if ta.tracker_role == ROLE_HMD => {
                hmd_id = Some(ta.tracker_id);
                tracing::info!(id = ta.tracker_id, "feeder HMD tracker added");
            }
            Some(protobuf_message::Message::Position(pos)) if Some(pos.tracker_id) == hmd_id => {
                let pose: HmdPose = [pos.x, pos.y, pos.z, pos.qx, pos.qy, pos.qz, pos.qw];
                *hmd.write().unwrap() = Some(pose);
            }
            _ => {}
        }
    }
    Ok(())
}

/// Read one length-prefixed message. Returns `None` on clean EOF.
async fn read_message<R: AsyncRead + Unpin>(r: &mut R) -> io::Result<Option<Vec<u8>>> {
    let mut len_buf = [0u8; 4];
    match r.read_exact(&mut len_buf).await {
        Ok(_) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let len = u32::from_le_bytes(len_buf) as usize;
    if !(4..=1024 * 1024).contains(&len) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("bad feeder message length {len}"),
        ));
    }
    let mut body = vec![0u8; len - 4];
    r.read_exact(&mut body).await?;
    Ok(Some(body))
}

// Silence the unused-import lint for `AsyncWrite` (imported for symmetry with
// future writes; only reads are needed today).
#[allow(unused_imports)]
use AsyncWrite as _;
