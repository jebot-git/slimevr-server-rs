//! Dump the SolarXR bone feed from a server (default: the Java SlimeVR server).
//!
//! Connects to a SolarXR WebSocket, requests a `bone_mask` data feed, and prints
//! each received bone's body part, rotation, head position, and length. Used for
//! parity-testing the Rust server against the Java server.
//!
//! ```text
//! cargo run --example solarxr_dump [ws://127.0.0.1:21110]
//! ```

use futures_util::{SinkExt, StreamExt};
use solarxr_protocol::data_feed::{
    Bone, DataFeedConfig, DataFeedConfigArgs, DataFeedMessage, DataFeedMessageHeader,
    DataFeedMessageHeaderArgs, StartDataFeed, StartDataFeedArgs,
};
use solarxr_protocol::flatbuffers::{self, FlatBufferBuilder};
use solarxr_protocol::{MessageBundle, MessageBundleArgs};
use tokio_tungstenite::tungstenite::Message;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let url = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "ws://127.0.0.1:21110".to_string());

    let (ws, _) = tokio_tungstenite::connect_async(&url).await?;
    eprintln!("connected to {url}");
    let (mut write, mut read) = ws.split();

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
        write
            .send(Message::Binary(fbb.finished_data().to_vec()))
            .await?;
    }

    while let Some(msg) = read.next().await {
        match msg? {
            Message::Binary(data) => {
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
            Message::Close(_) => break,
            _ => {}
        }
    }
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
