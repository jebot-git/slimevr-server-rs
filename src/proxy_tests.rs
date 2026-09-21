use super::*;
use solarxr_protocol::{MessageBundleArgs,
    data_feed::{DataFeedMessageHeader, DataFeedMessageHeaderArgs, StartDataFeed, StartDataFeedArgs},
    rpc::{ResetRequest, ResetRequestArgs, ResetType, RpcMessageHeader, RpcMessageHeaderArgs}};

fn setup(with_reset: bool) -> Vec<u8> {
    let mut b = flatbuffers::FlatBufferBuilder::new();
    let start = StartDataFeed::create(&mut b, &StartDataFeedArgs::default());
    let header = DataFeedMessageHeader::create(&mut b, &DataFeedMessageHeaderArgs {
        message_type: DataFeedMessage::StartDataFeed, message: Some(start.as_union_value()),
    });
    let feed = b.create_vector(&[header]);
    let rpc = if with_reset {
        let reset = ResetRequest::create(&mut b, &ResetRequestArgs { reset_type: ResetType::Full, ..Default::default() });
        let header = RpcMessageHeader::create(&mut b, &RpcMessageHeaderArgs {
            message_type: RpcMessage::ResetRequest, message: Some(reset.as_union_value()), ..Default::default()
        });
        Some(b.create_vector(&[header]))
    } else { None };
    let root = MessageBundle::create(&mut b, &MessageBundleArgs { data_feed_msgs: Some(feed), rpc_msgs: rpc, ..Default::default() });
    b.finish(root, None);
    b.finished_data().to_vec()
}

fn protobuf(message: proto::protobuf_message::Message) -> Vec<u8> {
    proto::ProtobufMessage { message: Some(message) }.encode_to_vec()
}
fn added(id: i32) -> Vec<u8> {
    protobuf(proto::protobuf_message::Message::TrackerAdded(proto::TrackerAdded { tracker_id: id, tracker_role: 19, ..Default::default() }))
}
fn position(id: i32, x: f32) -> Vec<u8> {
    protobuf(proto::protobuf_message::Message::Position(proto::Position { tracker_id: id, x, qw: 1.0, ..Default::default() }))
}

#[test]
fn never_replay_reset_even_when_bundled_with_subscription() {
    let mut cache = Cache::new(Protocol::SolarXr);
    assert!(!cache.observe(&setup(true)).unwrap());
    assert!(cache.replay().is_empty());
    let start = setup(false);
    for _ in 0..100 { assert!(cache.observe(&start).unwrap()); }
    assert_eq!(cache.replay(), vec![start]);
}

#[test]
fn feeder_replays_identities_before_only_latest_fresh_pose() {
    let mut cache = Cache::new(Protocol::Feeder);
    cache.observe(&added(4)).unwrap();
    cache.observe(&added(8)).unwrap();
    for i in 0..2000 { cache.observe(&position(4, i as f32)).unwrap(); }
    cache.observe(&position(8, 9.0)).unwrap();
    assert_eq!(cache.replay(), vec![added(4), added(8), position(4, 1999.0), position(8, 9.0)]);
    if let Cache::Feeder { trackers, .. } = &mut cache {
        trackers.get_mut(&4).unwrap().pose.as_mut().unwrap().0 = Instant::now() - Duration::from_secs(2);
    }
    assert_eq!(cache.replay(), vec![added(4), added(8), position(8, 9.0)]);
    let offline = protobuf(proto::protobuf_message::Message::TrackerStatus(proto::TrackerStatus { tracker_id: 8, status: 0 }));
    cache.observe(&offline).unwrap();
    assert_eq!(cache.replay(), vec![added(4), added(8), offline.clone()]);
    cache.observe(&position(8, 10.0)).unwrap();
    assert_eq!(cache.replay(), vec![added(4), added(8), offline]);
    let online = protobuf(proto::protobuf_message::Message::TrackerStatus(proto::TrackerStatus { tracker_id: 8, status: 1 }));
    cache.observe(&online).unwrap();
    assert_eq!(cache.replay(), vec![added(4), added(8), online, position(8, 10.0)]);
    // Re-announcing an id must not replay its previous owner's pose/status.
    cache.observe(&added(8)).unwrap();
    assert_eq!(cache.replay(), vec![added(4), added(8)]);
}

#[test]
fn feeder_identity_cache_is_bounded() {
    let mut cache = Cache::new(Protocol::Feeder);
    for i in 0..64 { cache.observe(&added(i)).unwrap(); }
    assert!(cache.observe(&added(64)).is_err());
    assert!(cache.observe(&[0xff]).is_err());
}

#[tokio::test]
async fn fragmented_reader_survives_unrelated_select_wakeups() {
    let (mut write, read) = tokio::io::duplex(128);
    let mut reader = Reader::spawn(read);
    let body = setup(false);
    let bytes = [((body.len() + 4) as u32).to_le_bytes().to_vec(), body.clone()].concat();
    // Leave the final byte unwritten so every receive future is cancelled by
    // another event while the dedicated reader is retaining a partial frame.
    for chunk in bytes[..bytes.len() - 1].chunks(3) {
        write.write_all(chunk).await.unwrap();
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_millis(1)) => {},
            frame = reader.rx.recv() => panic!("unexpected {frame:?}"),
        }
    }
    write.write_all(&bytes[bytes.len() - 1..]).await.unwrap();
    assert_eq!(reader.rx.recv().await.unwrap().unwrap(), body);
    drop(write);
    assert!(reader.rx.recv().await.is_none());
}

#[tokio::test]
async fn invalid_and_truncated_frames_are_rejected() {
    for input in [vec![1, 0, 0, 0], vec![0xff; 4], vec![8, 0, 0, 0, 1, 2]] {
        let mut slice = &input[..];
        assert!(read_frame(&mut slice).await.is_err());
    }
    assert!(read_frame(&mut &[][..]).await.unwrap().is_none());
}
