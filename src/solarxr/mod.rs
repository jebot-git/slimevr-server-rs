//! SolarXR WebSocket server — the WiVRn-facing skeleton output.
//!
//! WiVRn's built-in SolarXR driver connects here (port 21110) as a SolarXR
//! *client*. SlimeVR-Rust's `solarxr` crate is client-only, so this module
//! implements the *server* half from scratch on top of the vendored
//! `solarxr_protocol` FlatBuffers bindings.
//!
//! **Status (first milestone)**: accepts connections, verifies incoming
//! `MessageBundle`s (so we can see WiVRn's `StartDataFeed` request), and streams a
//! `DataFeedUpdate` with one `TrackerData` per tracked bone. TODO: the full
//! handshake (topic-mapping + `StartDataFeed` RPC reply) and switching from
//! synthetic trackers to the `bone_mask` bone feed WiVRn actually subscribes to.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use nalgebra::UnitQuaternion;
use solarxr_protocol::data_feed::tracker::{TrackerData, TrackerDataArgs};
use solarxr_protocol::data_feed::{
    DataFeedMessage, DataFeedMessageHeader, DataFeedMessageHeaderArgs, DataFeedUpdate,
    DataFeedUpdateArgs,
};
use solarxr_protocol::datatypes::math::Quat;
use solarxr_protocol::datatypes::{TrackerId, TrackerIdArgs};
use solarxr_protocol::flatbuffers;
use solarxr_protocol::{MessageBundle, MessageBundleArgs};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

/// The current skeleton pose: SolarXR `BodyPart` id → rotation.
pub type Pose = HashMap<u8, UnitQuaternion<f32>>;

/// Run the SolarXR WebSocket server forever.
pub async fn run(bind: SocketAddr, pose: Arc<RwLock<Pose>>) -> anyhow::Result<()> {
    let listener = TcpListener::bind(bind).await?;
    tracing::info!("SolarXR WebSocket server listening on {bind}");

    loop {
        let (stream, peer) = listener.accept().await?;
        tracing::info!(%peer, "SolarXR client connected");
        let pose = pose.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(stream, peer, pose).await {
                tracing::warn!(%peer, "SolarXR connection ended: {e:#}");
            }
        });
    }
}

/// Serve one WiVRn connection: read its requests, stream the current pose.
async fn handle_connection(
    stream: tokio::net::TcpStream,
    peer: SocketAddr,
    pose: Arc<RwLock<Pose>>,
) -> anyhow::Result<()> {
    let ws = tokio_tungstenite::accept_async(stream).await?;
    let (mut write, mut read) = ws.split();

    let mut tick = tokio::time::interval(Duration::from_millis(33)); // ~30 Hz
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            msg = read.next() => {
                match msg {
                    Some(Ok(Message::Binary(data))) => log_incoming(&data),
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Ok(_)) => {}
                    Some(Err(e)) => {
                        tracing::debug!(%peer, "SolarXR ws error: {e}");
                        break;
                    }
                }
            }
            _ = tick.tick() => {
                let pose = pose.read().unwrap().clone();
                if let Some(bytes) = build_data_feed_update(&pose) {
                    if write.send(Message::Binary(bytes)).await.is_err() {
                        break;
                    }
                }
            }
        }
    }

    Ok(())
}

/// Log a summary of an incoming SolarXR `MessageBundle`.
fn log_incoming(data: &[u8]) {
    match flatbuffers::root::<MessageBundle>(data) {
        Ok(bundle) => {
            let data_feed = bundle.data_feed_msgs().map(|v| v.len()).unwrap_or(0);
            let rpc = bundle.rpc_msgs().map(|v| v.len()).unwrap_or(0);
            let pubsub = bundle.pub_sub_msgs().map(|v| v.len()).unwrap_or(0);
            tracing::info!(
                "SolarXR MessageBundle: {data_feed} data_feed, {rpc} rpc, {pubsub} pub_sub msgs"
            );
        }
        Err(e) => tracing::debug!("non-SolarXR binary frame: {e}"),
    }
}

/// Build a `MessageBundle` containing a `DataFeedUpdate` with one `TrackerData`
/// per tracked body part.
fn build_data_feed_update(pose: &Pose) -> Option<Vec<u8>> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();

    let mut trackers = Vec::with_capacity(pose.len());
    for (&body_part, rot) in pose {
        let quat = Quat::new(rot.i, rot.j, rot.k, rot.w);
        let tracker_id = TrackerId::create(
            &mut fbb,
            &TrackerIdArgs {
                device_id: None,
                tracker_num: body_part,
            },
        );
        let tracker = TrackerData::create(
            &mut fbb,
            &TrackerDataArgs {
                tracker_id: Some(tracker_id),
                rotation: Some(&quat),
                ..Default::default()
            },
        );
        trackers.push(tracker);
    }
    let trackers_vec = fbb.create_vector(&trackers);

    let update = DataFeedUpdate::create(
        &mut fbb,
        &DataFeedUpdateArgs {
            synthetic_trackers: Some(trackers_vec),
            ..Default::default()
        },
    );

    // Wrap the update table as a union value inside a DataFeedMessageHeader.
    let msg_offset = flatbuffers::WIPOffset::new(update.value());
    let header = DataFeedMessageHeader::create(
        &mut fbb,
        &DataFeedMessageHeaderArgs {
            message_type: DataFeedMessage::DataFeedUpdate,
            message: Some(msg_offset),
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

    Some(fbb.finished_data().to_vec())
}
