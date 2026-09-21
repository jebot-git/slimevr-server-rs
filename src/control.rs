//! Newline-delimited JSON control over an owner-only Unix socket.
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;
use std::os::unix::fs::{FileTypeExt, PermissionsExt, MetadataExt};
use serde::{Deserialize, Serialize};
use tokio::{io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader}, net::{UnixListener, UnixStream}, sync::{mpsc, oneshot, Semaphore}};
use crate::status::StatusHandle;

#[derive(Debug, Clone, Copy, Deserialize, PartialEq)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
pub enum Command {
    Status, YawReset, FullReset, MountingReset, PauseTracking, Restart, Shutdown,
    SetSettings { settings: crate::settings::TrackingSettings },
    ClearDrift,
}
pub struct Envelope {
    pub command: Command,
    pub reply: Option<oneshot::Sender<Result<(), String>>>,
}
impl From<Command> for Envelope { fn from(command: Command) -> Self { Self { command, reply: None } } }
pub type CommandSender = mpsc::Sender<Envelope>;

fn parse_request(bytes: &[u8]) -> anyhow::Result<Command> {
    // Serde's internally tagged unit variants ignore extra keys; validate the
    // request envelope separately so misspelled or irrelevant payloads fail.
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Request { command: String, settings: Option<crate::settings::TrackingSettings> }
    let request: Request = serde_json::from_slice(bytes)?;
    anyhow::ensure!(request.command == "set_settings" || request.settings.is_none(),
        "settings are only accepted for set_settings");
    Ok(serde_json::from_slice(bytes)?)
}

pub struct ControlServer {
    listener: UnixListener,
    path: PathBuf,
    inode: u64,
}
impl ControlServer {
    pub async fn bind(path: PathBuf) -> anyhow::Result<Self> {
        match std::fs::symlink_metadata(&path) {
            Ok(meta) => {
                anyhow::ensure!(meta.file_type().is_socket(), "control path is not a socket: {}", path.display());
                match UnixStream::connect(&path).await {
                    Ok(_) => anyhow::bail!("control socket is already in use: {}", path.display()),
                    Err(error) if error.kind() == std::io::ErrorKind::ConnectionRefused => std::fs::remove_file(&path)?,
                    Err(error) => return Err(error.into()),
                }
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
        let listener = UnixListener::bind(&path)?;
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600))?;
        let inode = std::fs::symlink_metadata(&path)?.ino();
        Ok(Self { listener, path, inode })
    }
    pub async fn run(self, status: StatusHandle, commands: CommandSender) -> anyhow::Result<()> {
        let permits = Arc::new(Semaphore::new(16));
        let mut clients = tokio::task::JoinSet::new();
        loop {
            tokio::select! {
                incoming = self.listener.accept() => {
                    let (stream, _) = incoming?;
                    if let Ok(permit) = permits.clone().try_acquire_owned() {
                        let (status, commands) = (status.clone(), commands.clone());
                        clients.spawn(async move {
                            let _permit = permit;
                            let _ = serve(stream, status, commands).await;
                        });
                    }
                }
                _ = clients.join_next(), if !clients.is_empty() => {}
            }
        }
    }
}
impl Drop for ControlServer {
    fn drop(&mut self) {
        if std::fs::symlink_metadata(&self.path).is_ok_and(|meta| meta.ino() == self.inode) {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

async fn serve(stream: UnixStream, status: StatusHandle, commands: CommandSender) -> anyhow::Result<()> {
    let (read, mut write) = stream.into_split();
    let mut reader = BufReader::new(read);
    loop {
        let mut line = Vec::new();
        // A missing newline cannot grow memory without bound.
        let count = tokio::time::timeout(Duration::from_secs(30),
            (&mut reader).take(4097).read_until(b'\n', &mut line)).await??;
        if count == 0 { return Ok(()); }
        anyhow::ensure!(count <= 4096 && line.last() == Some(&b'\n'), "invalid control frame");
        let result = match parse_request(&line) {
            Ok(Command::Status) => Ok(()),
            Ok(command) => {
                let (tx, rx) = oneshot::channel();
                commands.send(Envelope { command, reply: Some(tx) }).await?;
                tokio::time::timeout(Duration::from_secs(5), rx).await??
            }
            Err(error) => Err(error.to_string()),
        };
        #[derive(Serialize)]
        struct Response { ok: bool, error: Option<String>, status: crate::status::StatusSnapshot }
        let response = Response { ok: result.is_ok(), error: result.err(), status: status.read().unwrap().clone() };
        let mut bytes = serde_json::to_vec(&response)?; bytes.push(b'\n');
        tokio::time::timeout(Duration::from_secs(5), write.write_all(&bytes)).await??;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn settings_commands_are_typed_and_old_commands_remain_compatible() {
        assert_eq!(serde_json::from_str::<Command>(r#"{"command":"yaw_reset"}"#).unwrap(), Command::YawReset);
        let request = serde_json::json!({"command": "set_settings", "settings": crate::settings::TrackingSettings::default()});
        assert!(matches!(serde_json::from_value::<Command>(request).unwrap(), Command::SetSettings { .. }));
        assert!(serde_json::from_str::<Command>(r#"{"command":"set_settings"}"#).is_err());
        assert!(parse_request(br#"{"command":"yaw_reset","settings":{}}"#).is_err());
        assert!(parse_request(br#"{"command":"yaw_reset","unknown":true}"#).is_err());
    }
}
