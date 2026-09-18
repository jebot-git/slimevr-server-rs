//! SolarXR IPC server — the WiVRn-facing skeleton output.
//!
//! WiVRn's built-in SolarXR driver connects to the SlimeVR server over a Unix
//! domain socket (`/run/user/1000/SlimeVRRpc`) and speaks the SolarXR
//! `MessageBundle` protocol, framed as a 4-byte little-endian length prefix
//! (the length includes the 4 prefix bytes) followed by the FlatBuffers payload.
//! This mirrors the Java server's `UnixSocketRpcBridge` framing.
//!
//! Handshake: the client sends a `StartDataFeed` (with `bone_mask: true`) and may
//! send a `SubscriptionRequest`; we reply with a `TopicMapping` and then stream
//! `DataFeedUpdate` messages containing one `Bone` per body part.

use std::collections::HashMap;
use std::io;
use std::path::Path;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use firmware_protocol::ActionType;
use solarxr_protocol::data_feed::tracker::{TrackerData, TrackerDataArgs, TrackerInfo, TrackerInfoArgs};
use solarxr_protocol::data_feed::{
    Bone, BoneArgs, DataFeedMessage, DataFeedMessageHeader, DataFeedMessageHeaderArgs,
    DataFeedUpdate, DataFeedUpdateArgs,
};
use solarxr_protocol::datatypes::math::{Quat, Vec3f};
use solarxr_protocol::datatypes::{
    BodyPart, TrackerId, TrackerIdArgs, TrackerStatus,
};
use solarxr_protocol::flatbuffers;
use solarxr_protocol::pub_sub::{
    PubSubHeader, PubSubHeaderArgs, PubSubUnion, SubscriptionRequest, TopicHandle,
    TopicHandleArgs, TopicId, TopicIdArgs, TopicMapping, TopicMappingArgs,
};
use solarxr_protocol::rpc::{
    AutoBoneProcessRequest, AutoBoneProcessStatusResponse, AutoBoneProcessStatusResponseArgs,
    AutoBoneProcessType, ResetRequest, ResetResponse, ResetResponseArgs, ResetStatus, ResetType,
    RpcMessage, RpcMessageHeader, RpcMessageHeaderArgs, SettingsResponse, SettingsResponseArgs,
};
use solarxr_protocol::{MessageBundle, MessageBundleArgs};
use skeletal_model::BoneMap;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::net::{UnixListener, UnixStream};

use crate::autobone;
use crate::calibration::Calibration;
use crate::feeder::HmdPose;
use crate::reset;
use crate::skeleton::BonePose;
use crate::tracker::TrackerRegistry;

/// The current skeleton pose: SolarXR `BodyPart` id → solved bone pose.
pub type Pose = HashMap<u8, BonePose>;

/// Run the SolarXR IPC server (Unix domain socket) forever.
pub async fn run(
    path: String,
    pose: Arc<RwLock<Pose>>,
    registry: Arc<RwLock<TrackerRegistry>>,
    calib: Arc<RwLock<Calibration>>,
    hmd: Arc<RwLock<Option<HmdPose>>>,
    lengths: Arc<RwLock<BoneMap<f32>>>,
    autobone: Arc<RwLock<autobone::AutoboneState>>,
    target_height: f32,
) -> anyhow::Result<()> {
    let listener = bind_unix_socket(&path).await?;
    tracing::info!("SolarXR IPC socket listening on {path}");

    loop {
        let (stream, _peer) = listener.accept().await?;
        tracing::info!("SolarXR client connected");
        let pose = pose.clone();
        let registry = registry.clone();
        let calib = calib.clone();
        let hmd = hmd.clone();
        let lengths = lengths.clone();
        let autobone = autobone.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(
                stream,
                pose,
                registry,
                calib,
                hmd,
                lengths,
                autobone,
                target_height,
            )
            .await
            {
                tracing::warn!("SolarXR connection ended: {e:#}");
            }
        });
    }
}

/// Bind a Unix domain socket, removing a stale socket file if one is present.
async fn bind_unix_socket(path: &str) -> anyhow::Result<UnixListener> {
    let p = Path::new(path);
    if p.exists() {
        // If a live server owns the socket, a connect attempt succeeds; otherwise
        // the socket file is stale and safe to remove.
        match UnixStream::connect(p).await {
            Ok(_) => anyhow::bail!("SolarXR socket {path:?} is already in use"),
            Err(_) => {
                tracing::warn!("removing stale SolarXR socket {path:?}");
                std::fs::remove_file(p)?;
            }
        }
    }
    Ok(UnixListener::bind(p)?)
}

/// Serve one WiVRn connection: handle the handshake, stream the bone feed, and
/// react to RPC messages (e.g. resets).
async fn handle_connection(
    stream: UnixStream,
    pose: Arc<RwLock<Pose>>,
    registry: Arc<RwLock<TrackerRegistry>>,
    calib: Arc<RwLock<Calibration>>,
    hmd: Arc<RwLock<Option<HmdPose>>>,
    lengths: Arc<RwLock<BoneMap<f32>>>,
    autobone: Arc<RwLock<autobone::AutoboneState>>,
    target_height: f32,
) -> anyhow::Result<()> {
    let (mut read, mut write) = stream.into_split();

    let mut streaming = false;
    let mut tick = tokio::time::interval(Duration::from_millis(33)); // ~30 Hz
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            msg = read_message(&mut read) => {
                match msg {
                    Ok(Some(data)) => {
                        match flatbuffers::root::<MessageBundle>(&data) {
                            Ok(bundle) => {
                                // StartDataFeed → begin streaming; PollDataFeed → one-shot reply.
                                if let Some(msgs) = bundle.data_feed_msgs() {
                                    for i in 0..msgs.len() {
                                        let h = msgs.get(i);
                                        match h.message_type() {
                                            DataFeedMessage::StartDataFeed => {
                                                streaming = true;
                                                tracing::info!("SolarXR StartDataFeed received; streaming bones");
                                            }
                                            DataFeedMessage::PollDataFeed => {
                                                tracing::debug!("SolarXR PollDataFeed received; sending update");
                                                let pose = pose.read().unwrap().clone();
                                                let hmd = *hmd.read().unwrap();
                                                if let Some(bytes) = build_bone_feed(&pose, hmd.as_ref()) {
                                                    let _ = write_message(&mut write, &bytes).await;
                                                }
                                            }
                                            _ => {}
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
                                            let _ = write_message(&mut write, &bytes).await;
                                        }
                                    }
                                }
                                // RPC messages (reset + settings requests from the client).
                                if let Some(msgs) = bundle.rpc_msgs() {
                                    for i in 0..msgs.len() {
                                        let h = msgs.get(i);
                                        match h.message_type() {
                                            RpcMessage::ResetRequest => {
                                                if let Some(req) = h.message_as_reset_request() {
                                                    tracing::info!("RPC ResetRequest");
                                                    if let Some(bytes) =
                                                        handle_reset_request(req, &registry, &calib)
                                                    {
                                                        let _ = write_message(&mut write, &bytes).await;
                                                    }
                                                }
                                            }
                                            RpcMessage::SettingsRequest => {
                                                tracing::debug!("RPC SettingsRequest");
                                                let _ =
                                                    write_message(&mut write, &build_settings_response())
                                                        .await;
                                            }
                                            RpcMessage::AutoBoneProcessRequest => {
                                                if let Some(req) = h.message_as_auto_bone_process_request() {
                                                    tracing::info!("RPC AutoBoneProcessRequest");
                                                    if let Some(bytes) = handle_autobone_request(
                                                        req,
                                                        &autobone,
                                                        &lengths,
                                                        &calib,
                                                        target_height,
                                                    ) {
                                                        let _ = write_message(&mut write, &bytes).await;
                                                    }
                                                }
                                            }
                                            _ => {}
                                        }
                                    }
                                }
                            }
                            Err(e) => tracing::debug!("non-SolarXR frame: {e}"),
                        }
                    }
                    Ok(None) => break, // clean EOF
                    Err(e) => {
                        tracing::debug!("SolarXR read error: {e}");
                        break;
                    }
                }
            }
            _ = tick.tick() => {
                if streaming {
                    let pose = pose.read().unwrap().clone();
                    let hmd = *hmd.read().unwrap();
                    if let Some(bytes) = build_bone_feed(&pose, hmd.as_ref()) {
                        if write_message(&mut write, &bytes).await.is_err() {
                            break;
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

/// Read one length-prefixed message (4-byte little-endian length, which includes
/// the 4 prefix bytes, followed by the payload). Returns `None` on clean EOF.
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
            format!("bad SolarXR message length {len}"),
        ));
    }
    let mut body = vec![0u8; len - 4];
    r.read_exact(&mut body).await?;
    Ok(Some(body))
}

/// Write one length-prefixed message.
async fn write_message<W: AsyncWrite + Unpin>(w: &mut W, body: &[u8]) -> io::Result<()> {
    let len = (body.len() + 4) as u32;
    w.write_all(&len.to_le_bytes()).await?;
    w.write_all(body).await?;
    Ok(())
}

/// The computed 6-DoF trackers we emit as emulated Vive trackers:
/// `(tracker body part, source bone body part, use tail joint)`.
const COMPUTED_TRACKERS: &[(u8, u8, bool)] = &[
    (1, 2, false),  // HEAD ← Neck head (root at origin)
    (3, 3, false),  // CHEST ← Chest head
    (5, 5, true),   // HIP ← Hip tail
    (8, 6, true),   // LEFT_LOWER_LEG (knee) ← ThighL tail
    (9, 7, true),   // RIGHT_LOWER_LEG (knee) ← ThighR tail
    (10, 10, true), // LEFT_FOOT ← FootL tail
    (11, 11, true), // RIGHT_FOOT ← FootR tail
    (14, 16, true), // LEFT_LOWER_ARM (elbow) ← UpperArmL tail
    (15, 17, true), // RIGHT_LOWER_ARM (elbow) ← UpperArmR tail
    (18, 18, true), // LEFT_HAND ← WristL tail
    (19, 19, true), // RIGHT_HAND ← WristR tail
];

/// The child-side (tail) joint position of a bone.
fn bone_tail_pos(bp: &BonePose) -> [f32; 3] {
    let off = bp.rotation * nalgebra::Vector3::new(0.0, -bp.length, 0.0);
    [
        bp.head_pos[0] + off.x,
        bp.head_pos[1] + off.y,
        bp.head_pos[2] + off.z,
    ]
}

/// Build a `MessageBundle` containing a `DataFeedUpdate` with the bone feed and
/// the computed synthetic trackers (emulated Vive trackers).
fn build_bone_feed(pose: &Pose, hmd: Option<&HmdPose>) -> Option<Vec<u8>> {
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

    let mut trackers = Vec::with_capacity(COMPUTED_TRACKERS.len());
    for (i, &(tracker_part, src_bone, use_tail)) in COMPUTED_TRACKERS.iter().enumerate() {
        let Some(src) = pose.get(&src_bone) else {
            continue;
        };
        let mut pos = if use_tail {
            bone_tail_pos(src)
        } else {
            src.head_pos
        };
        // Anchor the emulated Vive trackers at the HMD position (if known).
        if let Some(h) = hmd {
            pos[0] += h[0];
            pos[1] += h[1];
            pos[2] += h[2];
        }
        let quat = Quat::new(src.rotation.i, src.rotation.j, src.rotation.k, src.rotation.w);
        let position = Vec3f::new(pos[0], pos[1], pos[2]);

        // `device_id` must be absent: WiVRn filters out trackers with a device id
        // ("loopback feeder devices"). `tracker_num` alone is the stable identity.
        let tracker_id = TrackerId::create(
            &mut fbb,
            &TrackerIdArgs {
                device_id: None,
                tracker_num: i as u8,
            },
        );
        let info = TrackerInfo::create(
            &mut fbb,
            &TrackerInfoArgs {
                body_part: BodyPart(tracker_part),
                is_computed: true,
                ..Default::default()
            },
        );
        let tracker = TrackerData::create(
            &mut fbb,
            &TrackerDataArgs {
                tracker_id: Some(tracker_id),
                info: Some(info),
                status: TrackerStatus::OK,
                rotation: Some(&quat),
                position: Some(&position),
                ..Default::default()
            },
        );
        trackers.push(tracker);
    }
    let trackers_vec = fbb.create_vector(&trackers);

    let update = DataFeedUpdate::create(
        &mut fbb,
        &DataFeedUpdateArgs {
            bones: Some(bones_vec),
            synthetic_trackers: Some(trackers_vec),
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

/// Apply an RPC reset request and build the matching `ResetResponse`.
fn handle_reset_request(
    req: ResetRequest<'_>,
    registry: &Arc<RwLock<TrackerRegistry>>,
    calib: &Arc<RwLock<Calibration>>,
) -> Option<Vec<u8>> {
    let action = match req.reset_type() {
        ResetType::Yaw => ActionType::ResetYaw,
        ResetType::Full => ActionType::Reset,
        ResetType::Mounting => ActionType::ResetMounting,
        _ => return None,
    };
    {
        let reg = registry.read().unwrap();
        let mut cal = calib.write().unwrap();
        reset::handle_user_action(&reg, &mut cal, &action);
    }
    Some(build_reset_response(req.reset_type()))
}

/// Build a `MessageBundle` containing an `RpcMessageHeader` + `ResetResponse`.
fn build_reset_response(reset_type: ResetType) -> Vec<u8> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();
    let resp = ResetResponse::create(
        &mut fbb,
        &ResetResponseArgs {
            reset_type,
            status: ResetStatus::FINISHED,
            body_parts: None,
            progress: 0,
            duration: 0,
        },
    );
    let header = RpcMessageHeader::create(
        &mut fbb,
        &RpcMessageHeaderArgs {
            tx_id: None,
            message_type: RpcMessage::ResetResponse,
            message: Some(flatbuffers::WIPOffset::new(resp.value())),
        },
    );
    let msgs = fbb.create_vector(&[header]);
    let bundle = MessageBundle::create(
        &mut fbb,
        &MessageBundleArgs {
            rpc_msgs: Some(msgs),
            ..Default::default()
        },
    );
    fbb.finish(bundle, None);
    fbb.finished_data().to_vec()
}

/// Build an (empty) `SettingsResponse` — enough to satisfy the client's
/// `SettingsRequest` during the handshake.
fn build_settings_response() -> Vec<u8> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();
    let resp = SettingsResponse::create(&mut fbb, &SettingsResponseArgs::default());
    let header = RpcMessageHeader::create(
        &mut fbb,
        &RpcMessageHeaderArgs {
            tx_id: None,
            message_type: RpcMessage::SettingsResponse,
            message: Some(flatbuffers::WIPOffset::new(resp.value())),
        },
    );
    let msgs = fbb.create_vector(&[header]);
    let bundle = MessageBundle::create(
        &mut fbb,
        &MessageBundleArgs {
            rpc_msgs: Some(msgs),
            ..Default::default()
        },
    );
    fbb.finish(bundle, None);
    fbb.finished_data().to_vec()
}

/// Handle an autobone record/process request and build a status response.
fn handle_autobone_request(
    req: AutoBoneProcessRequest<'_>,
    autobone: &Arc<RwLock<autobone::AutoboneState>>,
    lengths: &Arc<RwLock<BoneMap<f32>>>,
    calib: &Arc<RwLock<Calibration>>,
    target_height: f32,
) -> Option<Vec<u8>> {
    let ptype = req.process_type();
    match ptype {
        AutoBoneProcessType::RECORD => {
            let mut ab = autobone.write().unwrap();
            ab.recording = true;
            ab.frames.clear();
            tracing::info!("autobone recording started");
        }
        AutoBoneProcessType::PROCESS => {
            let mut ab = autobone.write().unwrap();
            ab.recording = false;
            let frames = std::mem::take(&mut ab.frames);
            drop(ab);
            if frames.is_empty() {
                tracing::warn!("autobone: no frames recorded");
            } else {
                let calib = calib.read().unwrap();
                let cur = *lengths.read().unwrap();
                let out = autobone::optimize(&frames, &calib, cur, target_height, 20);
                *lengths.write().unwrap() = out;
                tracing::info!(frames = frames.len(), "autobone processed");
            }
        }
        AutoBoneProcessType::APPLY
        | AutoBoneProcessType::SAVE
        | AutoBoneProcessType::NONE => {}
        _ => {}
    }
    Some(build_autobone_status_response(ptype))
}

/// Build an `AutoBoneProcessStatusResponse` (completed + success).
fn build_autobone_status_response(ptype: AutoBoneProcessType) -> Vec<u8> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();
    let resp = AutoBoneProcessStatusResponse::create(
        &mut fbb,
        &AutoBoneProcessStatusResponseArgs {
            process_type: ptype,
            current: 1,
            total: 1,
            completed: true,
            success: true,
            eta: 0.0,
        },
    );
    let header = RpcMessageHeader::create(
        &mut fbb,
        &RpcMessageHeaderArgs {
            tx_id: None,
            message_type: RpcMessage::AutoBoneProcessStatusResponse,
            message: Some(flatbuffers::WIPOffset::new(resp.value())),
        },
    );
    let msgs = fbb.create_vector(&[header]);
    let bundle = MessageBundle::create(
        &mut fbb,
        &MessageBundleArgs {
            rpc_msgs: Some(msgs),
            ..Default::default()
        },
    );
    fbb.finish(bundle, None);
    fbb.finished_data().to_vec()
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

        let bytes = build_bone_feed(&pose, None).unwrap();
        let bundle = flatbuffers::root::<MessageBundle>(&bytes).unwrap();

        let msgs = bundle.data_feed_msgs().unwrap();
        assert_eq!(msgs.len(), 1);
        let header = msgs.get(0);
        assert_eq!(header.message_type(), DataFeedMessage::DataFeedUpdate);

        let update = header.message_as_data_feed_update().unwrap();
        let bones = update.bones().unwrap();
        assert_eq!(bones.len(), 1);
        assert_eq!(bones.get(0).body_part(), BodyPart(3));

        // The chest bone also produces one computed (synthetic) tracker.
        let trackers = update.synthetic_trackers().unwrap();
        assert_eq!(trackers.len(), 1);
        let td = trackers.get(0);
        assert_eq!(td.status(), TrackerStatus::OK);
        assert_eq!(td.info().unwrap().body_part(), BodyPart(3));
        assert!(td.info().unwrap().is_computed());
        assert!(td.position().is_some());
    }

    #[test]
    fn reset_response_round_trips() {
        let bytes = build_reset_response(ResetType::Full);
        let bundle = flatbuffers::root::<MessageBundle>(&bytes).unwrap();

        let msgs = bundle.rpc_msgs().unwrap();
        assert_eq!(msgs.len(), 1);
        let header = msgs.get(0);
        assert_eq!(header.message_type(), RpcMessage::ResetResponse);

        let resp = header.message_as_reset_response().unwrap();
        assert_eq!(resp.reset_type(), ResetType::Full);
        assert_eq!(resp.status(), ResetStatus::FINISHED);
    }

    #[test]
    fn settings_response_round_trips() {
        let bytes = build_settings_response();
        let bundle = flatbuffers::root::<MessageBundle>(&bytes).unwrap();

        let msgs = bundle.rpc_msgs().unwrap();
        assert_eq!(msgs.len(), 1);
        let header = msgs.get(0);
        assert_eq!(header.message_type(), RpcMessage::SettingsResponse);
        assert!(header.message_as_settings_response().is_some());
    }

    #[tokio::test]
    async fn message_framing_round_trips() {
        let (mut a, mut b) = tokio::io::duplex(1024);
        let payload = b"hello solarxr".to_vec();
        write_message(&mut a, &payload).await.unwrap();
        let got = read_message(&mut b).await.unwrap().unwrap();
        assert_eq!(got, payload);
    }
}
