//! SolarXR WebSocket server — the WiVRn-facing skeleton output.
//!
//! WiVRn's built-in SolarXR driver connects here (port 21110) as a SolarXR
//! *client*. SlimeVR-Rust's `solarxr` crate is client-only, so this module
//! implements the *server* half from scratch on top of the vendored
//! `solarxr_protocol` FlatBuffers bindings.
//!
//! Handshake: the client sends a `StartDataFeed` (with `bone_mask: true`) and may
//! send a `SubscriptionRequest`; we reply with a `TopicMapping` and then stream
//! `DataFeedUpdate` messages containing one `Bone` per body part.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use solarxr_protocol::data_feed::{
    Bone, BoneArgs, DataFeedMessage, DataFeedMessageHeader, DataFeedMessageHeaderArgs,
    DataFeedUpdate, DataFeedUpdateArgs,
};
use solarxr_protocol::datatypes::math::{Quat, Vec3f};
use solarxr_protocol::datatypes::BodyPart;
use solarxr_protocol::flatbuffers;
use solarxr_protocol::pub_sub::{
    PubSubHeader, PubSubHeaderArgs, PubSubUnion, SubscriptionRequest, TopicHandle,
    TopicHandleArgs, TopicId, TopicIdArgs, TopicMapping, TopicMappingArgs,
};
use solarxr_protocol::{MessageBundle, MessageBundleArgs};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

use crate::skeleton::BonePose;

/// The current skeleton pose: SolarXR `BodyPart` id → solved bone pose.
pub type Pose = HashMap<u8, BonePose>;

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

/// Serve one WiVRn connection: handle the handshake and stream the bone feed.
async fn handle_connection(
    stream: tokio::net::TcpStream,
    peer: SocketAddr,
    pose: Arc<RwLock<Pose>>,
) -> anyhow::Result<()> {
    let ws = tokio_tungstenite::accept_async(stream).await?;
    let (mut write, mut read) = ws.split();

    let mut streaming = false;
    let mut tick = tokio::time::interval(Duration::from_millis(33)); // ~30 Hz
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            msg = read.next() => {
                match msg {
                    Some(Ok(Message::Binary(data))) => {
                        match flatbuffers::root::<MessageBundle>(&data) {
                            Ok(bundle) => {
                                // StartDataFeed → begin streaming.
                                if let Some(msgs) = bundle.data_feed_msgs() {
                                    for i in 0..msgs.len() {
                                        let h = msgs.get(i);
                                        if h.message_type() == DataFeedMessage::StartDataFeed {
                                            streaming = true;
                                            tracing::info!(%peer, "SolarXR StartDataFeed received; streaming bones");
                                        }
                                    }
                                }
                                // Subscription / handle request → reply with a TopicMapping.
                                if let Some(msgs) = bundle.pub_sub_msgs() {
                                    for i in 0..msgs.len() {
                                        let h = msgs.get(i);
                                        let reply = match h.u_type() {
                                            PubSubUnion::SubscriptionRequest => h
                                                .u_as_subscription_request()
                                                .and_then(|r| build_topic_mapping_for_request(r)),
                                            PubSubUnion::TopicHandleRequest => h
                                                .u_as_topic_handle_request()
                                                .and_then(|r| r.id())
                                                .and_then(|t| build_topic_mapping_for_topic_id(t)),
                                            _ => None,
                                        };
                                        if let Some(bytes) = reply {
                                            let _ = write.send(Message::Binary(bytes)).await;
                                        }
                                    }
                                }
                            }
                            Err(e) => tracing::debug!(%peer, "non-SolarXR binary frame: {e}"),
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Ok(_)) => {}
                    Some(Err(e)) => {
                        tracing::debug!(%peer, "SolarXR ws error: {e}");
                        break;
                    }
                }
            }
            _ = tick.tick() => {
                if streaming {
                    let pose = pose.read().unwrap().clone();
                    if let Some(bytes) = build_bone_feed(&pose) {
                        if write.send(Message::Binary(bytes)).await.is_err() {
                            break;
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

/// Build a `MessageBundle` containing a `DataFeedUpdate` with one `Bone` per body part.
fn build_bone_feed(pose: &Pose) -> Option<Vec<u8>> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();

    let mut bones = Vec::with_capacity(pose.len());
    for (&body_part, bp) in pose {
        let quat = Quat::new(bp.rotation.i, bp.rotation.j, bp.rotation.k, bp.rotation.w);
        let head = Vec3f::new(bp.head_pos[0], bp.head_pos[1], bp.head_pos[2]);
        let bone = Bone::create(
            &mut fbb,
            &BoneArgs {
                body_part: BodyPart(body_part),
                rotation_g: Some(&quat),
                bone_length: bp.length,
                head_position_g: Some(&head),
            },
        );
        bones.push(bone);
    }
    let bones_vec = fbb.create_vector(&bones);

    let update = DataFeedUpdate::create(
        &mut fbb,
        &DataFeedUpdateArgs {
            bones: Some(bones_vec),
            ..Default::default()
        },
    );

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

/// Build a `TopicMapping` reply for a `SubscriptionRequest`, echoing its topic id.
fn build_topic_mapping_for_request(req: SubscriptionRequest<'_>) -> Option<Vec<u8>> {
    let topic_id = req.topic_as_topic_id()?;
    build_topic_mapping_for_topic_id(topic_id)
}

/// Build a `TopicMapping` reply for a `TopicId`, assigning handle 1.
fn build_topic_mapping_for_topic_id(topic_id: TopicId<'_>) -> Option<Vec<u8>> {
    let org = topic_id.organization()?;
    let app = topic_id.app_name()?;
    let topic = topic_id.topic()?;

    let mut fbb = flatbuffers::FlatBufferBuilder::new();
    let org = fbb.create_string(org);
    let app = fbb.create_string(app);
    let topic = fbb.create_string(topic);
    let topic_id = TopicId::create(
        &mut fbb,
        &TopicIdArgs {
            organization: Some(org),
            app_name: Some(app),
            topic: Some(topic),
        },
    );
    let handle = TopicHandle::create(&mut fbb, &TopicHandleArgs { id: 1 });
    let mapping = TopicMapping::create(
        &mut fbb,
        &TopicMappingArgs {
            id: Some(topic_id),
            handle: Some(handle),
        },
    );
    let header = PubSubHeader::create(
        &mut fbb,
        &PubSubHeaderArgs {
            u_type: PubSubUnion::TopicMapping,
            u: Some(flatbuffers::WIPOffset::new(mapping.value())),
        },
    );
    let msgs = fbb.create_vector(&[header]);
    let bundle = MessageBundle::create(
        &mut fbb,
        &MessageBundleArgs {
            pub_sub_msgs: Some(msgs),
            ..Default::default()
        },
    );
    fbb.finish(bundle, None);

    Some(fbb.finished_data().to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    #[test]
    fn bone_feed_round_trips() {
        let mut pose = Pose::new();
        pose.insert(
            3, // CHEST
            BonePose {
                rotation: UnitQuaternion::identity(),
                head_pos: [0.0, 1.5, 0.0],
                length: 0.22,
            },
        );

        let bytes = build_bone_feed(&pose).unwrap();
        let bundle = flatbuffers::root::<MessageBundle>(&bytes).unwrap();

        let msgs = bundle.data_feed_msgs().unwrap();
        assert_eq!(msgs.len(), 1);
        let header = msgs.get(0);
        assert_eq!(header.message_type(), DataFeedMessage::DataFeedUpdate);

        let update = header.message_as_data_feed_update().unwrap();
        let bones = update.bones().unwrap();
        assert_eq!(bones.len(), 1);
        assert_eq!(bones.get(0).body_part(), BodyPart(3));
    }
}
