//! Protocol-aware, bounded reconnect proxy for the native Rust backend.
//!
//! Only setup messages are replayed. Poses are coalesced while disconnected;
//! calibration/RPC actions are never queued across a backend outage.
use std::{collections::BTreeMap, io, path::{Path, PathBuf}, sync::Arc, time::{Duration, Instant}};
use std::os::unix::fs::{FileTypeExt, MetadataExt, PermissionsExt};
use anyhow::{Context, ensure};
use clap::Parser;
use prost::Message;
use solarxr_protocol::{flatbuffers, MessageBundle, data_feed::DataFeedMessage,
    pub_sub::PubSubUnion, rpc::RpcMessage};
use tokio::{io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt},
    net::{UnixListener, UnixStream, unix::OwnedWriteHalf},
    sync::{mpsc, Semaphore}, task::JoinHandle};

mod proto { include!(concat!(env!("OUT_DIR"), "/messages.rs")); }
const MAX_FRAME: usize = 1024 * 1024;
const MAX_CACHE: usize = 2 * MAX_FRAME;
const RETRY: Duration = Duration::from_millis(250);
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);
const POSE_MAX_AGE: Duration = Duration::from_secs(1);

#[derive(Parser, Debug)]
#[command(about = "Keep WiVRn's SolarXR and HMD feeder sockets alive across native tracking-server restarts")]
pub struct Options {
    /// Socket directory (defaults to XDG_RUNTIME_DIR, then /run/user/<uid>).
    #[arg(long)]
    runtime_dir: Option<PathBuf>,
    #[arg(long)]
    solarxr_listen: Option<PathBuf>,
    #[arg(long)]
    feeder_listen: Option<PathBuf>,
    #[arg(long)]
    solarxr_backend: Option<PathBuf>,
    #[arg(long)]
    feeder_backend: Option<PathBuf>,
}

#[derive(Clone, Copy, Debug)]
enum Protocol { SolarXr, Feeder }

/// Refuse occupied or non-socket paths. Only remove the socket inode we own.
struct Endpoint { listener: UnixListener, path: PathBuf, inode: u64 }
impl Endpoint {
    async fn bind(path: PathBuf) -> anyhow::Result<Self> {
        match std::fs::symlink_metadata(&path) {
            Ok(meta) => {
                ensure!(meta.file_type().is_socket(), "not a socket: {}", path.display());
                ensure!(meta.uid() == unsafe { libc::geteuid() }, "socket belongs to another user: {}", path.display());
                match UnixStream::connect(&path).await {
                    Ok(_) => anyhow::bail!("socket is already in use: {}", path.display()),
                    Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => std::fs::remove_file(&path)?,
                    Err(e) => return Err(e.into()),
                }
            }
            Err(e) if e.kind() == io::ErrorKind::NotFound => {},
            Err(e) => return Err(e.into()),
        }
        let listener = UnixListener::bind(&path)?;
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600))?;
        let inode = std::fs::symlink_metadata(&path)?.ino();
        Ok(Self { listener, path, inode })
    }
}
impl Drop for Endpoint {
    fn drop(&mut self) {
        if std::fs::symlink_metadata(&self.path).is_ok_and(|m| m.ino() == self.inode) {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

fn normalized_path(path: &Path) -> anyhow::Result<PathBuf> {
    // Canonicalize the parent even before the socket itself exists, so aliases
    // cannot turn a backend path into a self-connection loop.
    let absolute = if path.is_absolute() { path.to_owned() } else { std::env::current_dir()?.join(path) };
    if let Ok(meta) = std::fs::symlink_metadata(&absolute) {
        ensure!(!meta.file_type().is_symlink(), "socket paths must not be symlinks: {}", absolute.display());
    }
    Ok(absolute.parent().context("socket needs a parent directory")?.canonicalize()?
        .join(absolute.file_name().context("socket needs a filename")?))
}

pub async fn run(options: Options) -> anyhow::Result<()> {
    let runtime = options.runtime_dir.or_else(|| std::env::var_os("XDG_RUNTIME_DIR").map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from(format!("/run/user/{}", unsafe { libc::geteuid() })));
    let paths = [options.solarxr_listen.unwrap_or_else(|| runtime.join("SlimeVRRpc")),
        options.feeder_listen.unwrap_or_else(|| runtime.join("SlimeVRInput")),
        options.solarxr_backend.unwrap_or_else(|| runtime.join("SlimeVRRpc.backend")),
        options.feeder_backend.unwrap_or_else(|| runtime.join("SlimeVRInput.backend"))];
    let paths = paths.iter().map(|p| normalized_path(p)).collect::<anyhow::Result<Vec<_>>>()?;
    for i in 0..paths.len() { for j in 0..i { ensure!(paths[i] != paths[j], "all four socket paths must be different"); } }
    let solarxr = Endpoint::bind(paths[0].clone()).await?;
    let feeder = Endpoint::bind(paths[1].clone()).await?;
    tracing::info!(solarxr = %paths[0].display(), feeder = %paths[1].display(), "persistent proxy listening");
    let permits = Arc::new(Semaphore::new(16));
    let mut tasks = tokio::task::JoinSet::new();
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    loop {
        let accepted = tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            _ = terminate.recv() => break,
            _ = tasks.join_next(), if !tasks.is_empty() => continue,
            incoming = solarxr.listener.accept() => (incoming?, Protocol::SolarXr, paths[2].clone()),
            incoming = feeder.listener.accept() => (incoming?, Protocol::Feeder, paths[3].clone()),
        };
        if let Ok(permit) = permits.clone().try_acquire_owned() {
            tasks.spawn(async move {
                let _permit = permit;
                let ((stream, _), protocol, backend) = accepted;
                if let Err(error) = relay(stream, backend, protocol).await {
                    tracing::warn!(?protocol, %error, "proxy client closed");
                }
            });
        }
    }
    tasks.abort_all();
    while tasks.join_next().await.is_some() {}
    Ok(())
}

/// A dedicated reader retains partial frame bytes across select! iterations.
/// Dropping read_exact futures in a select loop would corrupt fragmented frames.
struct Reader { rx: mpsc::Receiver<io::Result<Vec<u8>>>, task: JoinHandle<()> }
impl Reader {
    fn spawn<R: AsyncRead + Unpin + Send + 'static>(mut read: R) -> Self {
        let (tx, rx) = mpsc::channel(4);
        let task = tokio::spawn(async move {
            loop {
                match read_frame(&mut read).await {
                    Ok(Some(frame)) => if tx.send(Ok(frame)).await.is_err() { break },
                    Ok(None) => break,
                    Err(error) => { let _ = tx.send(Err(error)).await; break; }
                }
            }
        });
        Self { rx, task }
    }
}
impl Drop for Reader { fn drop(&mut self) { self.task.abort(); } }
struct Backend { reader: Reader, write: OwnedWriteHalf }

async fn relay(client: UnixStream, path: PathBuf, protocol: Protocol) -> anyhow::Result<()> {
    let (read, mut client_write) = client.into_split();
    let mut client_read = Reader::spawn(read);
    let mut backend: Option<Backend> = None;
    let mut cache = Cache::new(protocol);
    let mut retry = tokio::time::interval(RETRY);
    retry.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    tracing::info!(?protocol, "proxy client connected");
    enum Event { Client(Option<io::Result<Vec<u8>>>), Backend(Option<io::Result<Vec<u8>>>), Retry }
    loop {
        let event = tokio::select! {
            msg = client_read.rx.recv() => Event::Client(msg),
            msg = async { match backend.as_mut() {
                Some(connection) => connection.reader.rx.recv().await,
                None => std::future::pending().await,
            }} => Event::Backend(msg),
            _ = retry.tick(), if backend.is_none() => Event::Retry,
        };
        match event {
            Event::Client(None) => break,
            Event::Client(Some(frame)) => {
                let frame = frame?;
                let replayable = cache.observe(&frame)?;
                if let Some(connection) = backend.as_mut() {
                    if write_frame(&mut connection.write, &frame).await.is_err() {
                        backend = None;
                        tracing::warn!(?protocol, "backend write failed; keeping client connected");
                    }
                } else if !replayable && matches!(protocol, Protocol::SolarXr) {
                    tracing::warn!("SolarXR action discarded while backend is unavailable; it will not be replayed");
                }
            }
            Event::Backend(Some(Ok(frame))) => write_frame(&mut client_write, &frame).await?,
            Event::Backend(_) => {
                backend = None;
                tracing::warn!(?protocol, "backend disconnected; keeping client connected");
            }
            Event::Retry => {
                if let Ok(Ok(stream)) = tokio::time::timeout(RETRY, UnixStream::connect(&path)).await {
                    let (read, mut write) = stream.into_split();
                    let reader = Reader::spawn(read);
                    let mut replay_ok = true;
                    for frame in cache.replay() {
                        if write_frame(&mut write, &frame).await.is_err() { replay_ok = false; break; }
                    }
                    if replay_ok {
                        backend = Some(Backend { reader, write });
                        tracing::info!(?protocol, path = %path.display(), "backend attached; setup restored");
                    }
                }
            }
        }
    }
    Ok(())
}

async fn read_frame<R: AsyncRead + Unpin>(read: &mut R) -> io::Result<Option<Vec<u8>>> {
    let mut header = [0; 4];
    if read.read(&mut header[..1]).await? == 0 { return Ok(None); }
    read.read_exact(&mut header[1..]).await?;
    let size = u32::from_le_bytes(header) as usize;
    if !(4..=MAX_FRAME).contains(&size) { return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid frame length")); }
    let mut frame = vec![0; size - 4];
    read.read_exact(&mut frame).await?;
    Ok(Some(frame))
}

async fn write_frame<W: AsyncWrite + Unpin>(write: &mut W, frame: &[u8]) -> io::Result<()> {
    tokio::time::timeout(WRITE_TIMEOUT, async {
        write.write_all(&((frame.len() + 4) as u32).to_le_bytes()).await?;
        write.write_all(frame).await
    }).await.map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "frame write timed out"))?
}

#[derive(Default)]
struct FeederTracker {
    added: Vec<u8>,
    status: Option<Vec<u8>>,
    inactive: bool,
    pose: Option<(Instant, Vec<u8>)>,
}
enum Cache {
    SolarXr(Vec<Vec<u8>>),
    Feeder { version: Option<Vec<u8>>, trackers: BTreeMap<i32, FeederTracker> },
}
impl Cache {
    fn new(protocol: Protocol) -> Self { match protocol {
        Protocol::SolarXr => Self::SolarXr(Vec::new()),
        Protocol::Feeder => Self::Feeder { version: None, trackers: BTreeMap::new() },
    } }
    fn observe(&mut self, frame: &[u8]) -> anyhow::Result<bool> {
        match self {
            Self::SolarXr(setup) => {
                let bundle = flatbuffers::root::<MessageBundle>(frame)?;
                // Cache only entire idempotent setup bundles. In particular a
                // reset bundled with StartDataFeed must NEVER be replayed.
                // WiVRn sends standalone StartDataFeed and Poll+Settings bundles.
                let safe = bundle.data_feed_msgs().is_none_or(|msgs| msgs.iter().all(|m|
                    matches!(m.message_type(), DataFeedMessage::StartDataFeed | DataFeedMessage::PollDataFeed)))
                    && bundle.rpc_msgs().is_none_or(|msgs| msgs.iter().all(|m| m.message_type() == RpcMessage::SettingsRequest))
                    && bundle.pub_sub_msgs().is_none_or(|msgs| msgs.iter().all(|m|
                        matches!(m.u_type(), PubSubUnion::SubscriptionRequest | PubSubUnion::TopicHandleRequest)));
                if safe && !setup.iter().any(|s| s == frame) {
                    ensure!(setup.len() < 32 && setup.iter().map(Vec::len).sum::<usize>() + frame.len() <= MAX_CACHE,
                        "SolarXR setup replay limit exceeded");
                    setup.push(frame.to_vec());
                }
                Ok(safe)
            }
            Self::Feeder { version, trackers } => {
                use proto::protobuf_message::Message::*;
                match proto::ProtobufMessage::decode(frame)?.message {
                    Some(Version(_)) => *version = Some(frame.to_vec()),
                    Some(TrackerAdded(added)) => {
                        ensure!(trackers.contains_key(&added.tracker_id) || trackers.len() < 64, "too many feeder trackers");
                        trackers.insert(added.tracker_id, FeederTracker { added: frame.to_vec(), ..Default::default() });
                    }
                    Some(Position(position)) => if let Some(tracker) = trackers.get_mut(&position.tracker_id) {
                        tracker.pose = Some((Instant::now(), frame.to_vec()));
                    },
                    Some(TrackerStatus(status)) => if let Some(tracker) = trackers.get_mut(&status.tracker_id) {
                        tracker.status = Some(frame.to_vec());
                        tracker.inactive = status.status != 1;
                        if tracker.inactive { tracker.pose = None; }
                    },
                    _ => return Ok(false),
                }
                let bytes = version.as_ref().map_or(0, Vec::len) + trackers.values().map(|t|
                    t.added.len() + t.status.as_ref().map_or(0, Vec::len) + t.pose.as_ref().map_or(0, |(_, p)| p.len())).sum::<usize>();
                ensure!(bytes <= MAX_CACHE, "feeder replay limit exceeded");
                Ok(true)
            }
        }
    }
    fn replay(&self) -> Vec<Vec<u8>> {
        match self {
            Self::SolarXr(setup) => setup.clone(),
            Self::Feeder { version, trackers } => {
                let mut frames: Vec<_> = version.iter().cloned().collect();
                // Announce every identity before replaying any pose.
                frames.extend(trackers.values().map(|t| t.added.clone()));
                for tracker in trackers.values() {
                    frames.extend(tracker.status.iter().cloned());
                    if let Some((at, pose)) = &tracker.pose {
                        if !tracker.inactive && at.elapsed() < POSE_MAX_AGE { frames.push(pose.clone()); }
                    }
                }
                frames
            }
        }
    }
}

#[cfg(test)]
#[path = "proxy_tests.rs"]
mod tests;
