//! Restart the real backend while simulated WiVRn clients keep their original sockets.
use std::{path::{Path, PathBuf}, process::{Child, Command, Stdio}, time::Duration};
use tokio::{io::{AsyncReadExt, AsyncWriteExt}, net::UnixStream};
use solarxr_protocol::{flatbuffers::{self, FlatBufferBuilder}, MessageBundle, MessageBundleArgs,
    data_feed::{DataFeedMessage, DataFeedMessageHeader, DataFeedMessageHeaderArgs, StartDataFeed, StartDataFeedArgs,
        DataFeedConfig, DataFeedConfigArgs, PollDataFeed, PollDataFeedArgs},
    rpc::{RpcMessage, RpcMessageHeader, RpcMessageHeaderArgs, SettingsRequest, SettingsRequestArgs}};

struct Process(Child);
impl Drop for Process { fn drop(&mut self) { let _ = self.0.kill(); let _ = self.0.wait(); } }
struct Directory(PathBuf);
impl Drop for Directory { fn drop(&mut self) { let _ = std::fs::remove_dir_all(&self.0); } }
fn directory() -> Directory {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!("shora-proxy-test-{}-{}", std::process::id(), NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)));
    std::fs::create_dir(&path).unwrap(); Directory(path)
}
fn start_backend(dir: &Path) -> Process {
    Process(Command::new(env!("CARGO_BIN_EXE_slimevr-server-rs"))
        .args(["--tracker-port", "0", "--no-face", "--solarxr-socket"]).arg(dir.join("SlimeVRRpc.backend"))
        .arg("--feeder-socket").arg(dir.join("SlimeVRInput.backend"))
        .arg("--control-socket").arg(dir.join("control"))
        .stdout(Stdio::null()).stderr(Stdio::null()).spawn().unwrap())
}
async fn connect(path: &Path) -> UnixStream {
    tokio::time::timeout(Duration::from_secs(5), async {
        loop { match UnixStream::connect(path).await {
            Ok(stream) => return stream,
            Err(_) => tokio::time::sleep(Duration::from_millis(20)).await,
        } }
    }).await.expect("socket became available")
}
async fn send(stream: &mut UnixStream, body: &[u8]) {
    stream.write_all(&((body.len() + 4) as u32).to_le_bytes()).await.unwrap();
    stream.write_all(body).await.unwrap();
}
async fn frame(stream: &mut UnixStream) -> Vec<u8> {
    let size = stream.read_u32_le().await.unwrap() as usize;
    assert!((4..=1024*1024).contains(&size));
    let mut bytes = vec![0; size - 4]; stream.read_exact(&mut bytes).await.unwrap(); bytes
}
fn start_feed() -> Vec<u8> {
    let mut b = FlatBufferBuilder::new();
    let config = DataFeedConfig::create(&mut b, &DataFeedConfigArgs { bone_mask: true, ..Default::default() });
    let configs = b.create_vector(&[config]);
    let start = StartDataFeed::create(&mut b, &StartDataFeedArgs { data_feeds: Some(configs) });
    let header = DataFeedMessageHeader::create(&mut b, &DataFeedMessageHeaderArgs {
        message_type: DataFeedMessage::StartDataFeed, message: Some(start.as_union_value()),
    });
    let headers = b.create_vector(&[header]);
    let root = MessageBundle::create(&mut b, &MessageBundleArgs { data_feed_msgs: Some(headers), ..Default::default() });
    b.finish(root, None); b.finished_data().to_vec()
}
fn initial_request() -> Vec<u8> {
    // WiVRn discovers settings/bones with this combined bundle before sending
    // its standalone StartDataFeed. Both must survive a backend replacement.
    let mut b = FlatBufferBuilder::new();
    let config = DataFeedConfig::create(&mut b, &DataFeedConfigArgs { bone_mask: true, ..Default::default() });
    let poll = PollDataFeed::create(&mut b, &PollDataFeedArgs { config: Some(config) });
    let feed = DataFeedMessageHeader::create(&mut b, &DataFeedMessageHeaderArgs {
        message_type: DataFeedMessage::PollDataFeed, message: Some(poll.as_union_value()),
    });
    let feeds = b.create_vector(&[feed]);
    let settings = SettingsRequest::create(&mut b, &SettingsRequestArgs::default());
    let rpc = RpcMessageHeader::create(&mut b, &RpcMessageHeaderArgs {
        message_type: RpcMessage::SettingsRequest, message: Some(settings.as_union_value()), ..Default::default()
    });
    let rpcs = b.create_vector(&[rpc]);
    let root = MessageBundle::create(&mut b, &MessageBundleArgs {
        data_feed_msgs: Some(feeds), rpc_msgs: Some(rpcs), ..Default::default()
    });
    b.finish(root, None); b.finished_data().to_vec()
}
fn hmd_position(x: f32) -> Vec<u8> {
    let mut position = vec![8, 4]; // tracker_id = 4
    for (field, value) in [(2, x), (3, 1.7_f32), (4, -0.4), (8, 1.0)] {
        position.push((field << 3) | 5); position.extend(value.to_le_bytes());
    }
    [vec![10, position.len() as u8], position].concat()
}
async fn assert_root(stream: &mut UnixStream, x: f32) {
    tokio::time::timeout(Duration::from_secs(5), async {
        let mut got_settings = false;
        let mut got_pose = false;
        loop {
            let bytes = frame(stream).await;
            let bundle = flatbuffers::root::<MessageBundle>(&bytes).unwrap();
            if let Some(rpcs) = bundle.rpc_msgs() {
                got_settings |= rpcs.iter().any(|h| h.message_type() == RpcMessage::SettingsResponse);
            }
            for header in bundle.data_feed_msgs().into_iter().flatten() {
                let update = header.message_as_data_feed_update().unwrap();
                for bone in update.bones().unwrap() {
                    let head = bone.head_position_g().unwrap();
                    if bone.body_part().0 == 2 && (head.x() - x).abs() < 1e-5 {
                        assert!((head.y() - 1.7).abs() < 1e-5); got_pose = true;
                    }
                }
            }
            if got_settings && got_pose { return; }
        }
    }).await.expect("original SolarXR socket resumed with current HMD pose");
}

#[tokio::test]
async fn real_backend_restarts_without_reconnecting_either_client() {
    let dir = directory();
    let _proxy = Process(Command::new(env!("CARGO_BIN_EXE_shora-proxy"))
        .arg("--runtime-dir").arg(&dir.0).stdout(Stdio::null()).stderr(Stdio::null()).spawn().unwrap());
    let mut solarxr = connect(&dir.0.join("SlimeVRRpc")).await;
    let mut feeder = connect(&dir.0.join("SlimeVRInput")).await;
    use std::os::unix::fs::PermissionsExt;
    assert_eq!(std::fs::metadata(dir.0.join("SlimeVRRpc")).unwrap().permissions().mode() & 0o777, 0o600);
    send(&mut solarxr, &initial_request()).await;
    send(&mut solarxr, &start_feed()).await;
    // Announce HMD once, even though the backend has not started yet.
    send(&mut feeder, &[26, 4, 8, 4, 32, 19]).await;
    let mut backend = start_backend(&dir.0);
    send(&mut feeder, &hmd_position(1.0)).await;
    assert_root(&mut solarxr, 1.0).await;
    for next in [2.0, -3.0, 0.75] {
        backend.0.kill().unwrap(); backend.0.wait().unwrap();
        // Continue producing poses while the backend is down. These must be
        // coalesced, never played back as a stale movement backlog.
        for i in 0..40 {
            send(&mut feeder, &hmd_position(next - 0.01 * (39 - i) as f32)).await;
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
        backend = start_backend(&dir.0);
        assert_root(&mut solarxr, next).await;
    }
    // Each new downstream session has its own cache; disconnected clients must
    // not leave replay state or reader tasks holding the backend open.
    drop(solarxr); drop(feeder);
    let mut fresh = connect(&dir.0.join("SlimeVRRpc")).await;
    assert!(tokio::time::timeout(Duration::from_millis(400), fresh.read_u8()).await.is_err());
}

#[tokio::test]
async fn proxy_refuses_occupied_and_non_socket_endpoints() {
    let dir = directory();
    std::fs::write(dir.0.join("SlimeVRInput"), "keep me").unwrap();
    let status = Command::new(env!("CARGO_BIN_EXE_shora-proxy"))
        .arg("--runtime-dir").arg(&dir.0).stdout(Stdio::null()).stderr(Stdio::null()).status().unwrap();
    assert!(!status.success());
    assert_eq!(std::fs::read_to_string(dir.0.join("SlimeVRInput")).unwrap(), "keep me");
    assert!(!dir.0.join("SlimeVRRpc").exists()); // First bind rolled back.
    std::fs::remove_file(dir.0.join("SlimeVRInput")).unwrap();
    let listener = tokio::net::UnixListener::bind(dir.0.join("SlimeVRRpc")).unwrap();
    let status = Command::new(env!("CARGO_BIN_EXE_shora-proxy"))
        .arg("--runtime-dir").arg(&dir.0).stdout(Stdio::null()).stderr(Stdio::null()).status().unwrap();
    assert!(!status.success());
    assert!(dir.0.join("SlimeVRRpc").exists());
    drop(listener);
    std::fs::remove_file(dir.0.join("SlimeVRRpc")).unwrap();
    // Reject a backend alias before accepting any clients (no proxy loop).
    std::os::unix::fs::symlink(dir.0.join("SlimeVRRpc"), dir.0.join("SlimeVRRpc.backend")).unwrap();
    let status = Command::new(env!("CARGO_BIN_EXE_shora-proxy"))
        .arg("--runtime-dir").arg(&dir.0).stdout(Stdio::null()).stderr(Stdio::null()).status().unwrap();
    assert!(!status.success());
    assert!(!dir.0.join("SlimeVRRpc").exists());
}
