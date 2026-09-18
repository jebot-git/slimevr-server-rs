//! Dump the SolarXR bone feed from a server over its Unix IPC socket.
//!
//! Connects to the SolarXR Unix domain socket, requests a `bone_mask` data feed,
//! and prints each received bone's body part, rotation, head position, and length.
//!
//! ```text
//! cargo run --example solarxr_dump [/run/user/1000/SlimeVRRpc]
//! ```

use solarxr_protocol::data_feed::{
    Bone, DataFeedConfig, DataFeedConfigArgs, DataFeedMessage, DataFeedMessageHeader,
    DataFeedMessageHeaderArgs, StartDataFeed, StartDataFeedArgs,
};
use solarxr_protocol::flatbuffers::{self, FlatBufferBuilder};
use solarxr_protocol::{MessageBundle, MessageBundleArgs};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/run/user/1000/SlimeVRRpc".to_string());

    let mut stream = UnixStream::connect(&path).await?;
    eprintln!("connected to {path}");

    // Send a StartDataFeed requesting bones.
    {
        let mut fbb = FlatBufferBuilder::new();
        let config = DataFeedConfig::create(
            &mut fbb,
            &DataFeedConfigArgs {
                minimum_time_since_last: 10,
                bone_mask: true,
                ..Default::default()
            },
        );
        let configs = fbb.create_vector(&[config]);
        let start = StartDataFeed::create(
            &mut fbb,
            &StartDataFeedArgs {
                data_feeds: Some(configs),
            },
        );
        let header = DataFeedMessageHeader::create(
            &mut fbb,
            &DataFeedMessageHeaderArgs {
                message_type: DataFeedMessage::StartDataFeed,
                message: Some(flatbuffers::WIPOffset::new(start.value())),
            },
        );
        let msgs = fbb.create_vector(&[header]);
        let bundle = MessageBundle::create(
            &mut fbb,
            &MessageBundleArgs {
                data_feed_msgs: Some(msgs),
                ..Default::default()
            },
        );
        fbb.finish(bundle, None);
        write_message(&mut stream, fbb.finished_data()).await?;
    }

    while let Some(data) = read_message(&mut stream).await? {
        if let Ok(bundle) = flatbuffers::root::<MessageBundle>(&data) {
            if let Some(msgs) = bundle.data_feed_msgs() {
                for i in 0..msgs.len() {
                    let h = msgs.get(i);
                    if h.message_type() == DataFeedMessage::DataFeedUpdate {
                        if let Some(update) = h.message_as_data_feed_update() {
                            if let Some(bones) = update.bones() {
                                for j in 0..bones.len() {
                                    print_bone(&bones.get(j));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

/// Read one length-prefixed message. Returns `None` on clean EOF.
async fn read_message(s: &mut UnixStream) -> std::io::Result<Option<Vec<u8>>> {
    let mut len_buf = [0u8; 4];
    match s.read_exact(&mut len_buf).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let len = u32::from_le_bytes(len_buf) as usize;
    let mut body = vec![0u8; len - 4];
    s.read_exact(&mut body).await?;
    Ok(Some(body))
}

/// Write one length-prefixed message.
async fn write_message(s: &mut UnixStream, body: &[u8]) -> std::io::Result<()> {
    let len = (body.len() + 4) as u32;
    s.write_all(&len.to_le_bytes()).await?;
    s.write_all(body).await?;
    Ok(())
}

fn print_bone(b: &Bone<'_>) {
    let name = b.body_part().variant_name().unwrap_or("UNKNOWN");
    let (qx, qy, qz, qw) = b
        .rotation_g()
        .map(|q| (q.x(), q.y(), q.z(), q.w()))
        .unwrap_or_default();
    let (px, py, pz) = b
        .head_position_g()
        .map(|p| (p.x(), p.y(), p.z()))
        .unwrap_or_default();
    println!(
        "{name:>16} rot=({qx:+.4},{qy:+.4},{qz:+.4},{qw:+.4}) pos=({px:+.4},{py:+.4},{pz:+.4}) len={:.4}",
        b.bone_length()
    );
}
