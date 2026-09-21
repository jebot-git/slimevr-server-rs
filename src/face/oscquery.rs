//! Local OSCQuery advertisement for the embedded relay.

use std::io::{Read, Write};
use std::net::{Ipv4Addr, TcpListener, TcpStream};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread::JoinHandle;
use std::time::Duration;

use anyhow::Context;
use mdns_sd::{ServiceDaemon, ServiceInfo};
use serde_json::{json, Value};

const NAME: &str = "SlimeVR-Rust-Face";

pub struct OscQueryAdvert {
    daemon: ServiceDaemon,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl OscQueryAdvert {
    pub fn new(osc_port: u16) -> anyhow::Result<Self> {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0))
            .context("bind OSCQuery HTTP listener")?;
        listener.set_nonblocking(true)?;
        let http_port = listener.local_addr()?.port();
        let daemon = ServiceDaemon::new()?;
        let services = [
            ("_osc._udp.local.", osc_port),
            ("_oscjson._tcp.local.", http_port),
        ];
        let stop = Arc::new(AtomicBool::new(false));
        let mut advert = Self { daemon, stop: stop.clone(), worker: None };
        advert.daemon.enable_interface(mdns_sd::IfKind::LoopbackV4)?;
        for (kind, port) in services {
            let info = ServiceInfo::new(kind, NAME, "slimevr-rust-face.local.",
                std::net::IpAddr::V4(Ipv4Addr::LOCALHOST), port, &[("txtvers", "1")][..])?;
            advert.daemon.register(info)?;
        }
        advert.worker = Some(std::thread::Builder::new().name("face-oscquery".into())
            .spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((stream, _)) => {
                            if let Err(error) = serve_request(stream, osc_port) {
                                tracing::debug!(%error, "OSCQuery HTTP request failed");
                            }
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            std::thread::park_timeout(Duration::from_millis(20));
                        }
                        Err(e) => {
                            tracing::warn!(%e, "OSCQuery HTTP listener stopped");
                            break;
                        }
                    }
                }
            })?);
        tracing::info!(osc_port, http_port, "face OSCQuery advertised on localhost");
        Ok(advert)
    }
}

impl Drop for OscQueryAdvert {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.daemon.shutdown();
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            let _ = worker.join();
        }
    }
}

fn node(path: &str, ty: &str) -> Value {
    json!({"FULL_PATH": path, "ACCESS": 2, "TYPE": ty})
}

fn response(path: &str, osc_port: u16) -> Option<Value> {
    let parameters = node("/avatar/parameters", "b");
    let avatar = json!({"FULL_PATH": "/avatar", "CONTENTS": {
        "change": node("/avatar/change", "s"), "parameters": parameters
    }});
    match path {
        "/?HOST_INFO" => Some(json!({"NAME": NAME, "OSC_IP": "127.0.0.1",
            "OSC_PORT": osc_port, "OSC_TRANSPORT": "UDP",
            "EXTENSIONS": {"ACCESS": true, "TYPE": true}})),
        "/" => Some(json!({"FULL_PATH": "/", "CONTENTS": {"avatar": avatar}})),
        "/avatar" => Some(avatar),
        "/avatar/change" => Some(node(path, "s")),
        "/avatar/parameters" => Some(parameters),
        _ => None,
    }
}

fn serve_request(mut stream: TcpStream, osc_port: u16) -> anyhow::Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(200)))?;
    stream.set_write_timeout(Some(Duration::from_millis(200)))?;
    let mut request = Vec::new();
    let deadline = std::time::Instant::now() + Duration::from_millis(400);
    loop {
        let mut buf = [0; 1024];
        let len = stream.read(&mut buf)?;
        if len == 0 { return Ok(()); }
        request.extend_from_slice(&buf[..len]);
        if request.windows(4).any(|s| s == b"\r\n\r\n") { break; }
        anyhow::ensure!(request.len() <= 8192 && std::time::Instant::now() < deadline,
            "incomplete OSCQuery HTTP request");
    }
    let text = std::str::from_utf8(&request)?;
    let path = text.lines().next().and_then(|s| s.split_whitespace().nth(1)).unwrap_or("");
    let body = response(path, osc_port);
    let status = if body.is_some() { "200 OK" } else { "404 Not Found" };
    let body = body.unwrap_or_else(|| json!({})).to_string();
    write!(stream, "HTTP/1.1 {status}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}", body.len())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovery_tree_and_host_info_describe_actual_relay() {
        let host = response("/?HOST_INFO", 12345).unwrap();
        assert_eq!(host["OSC_PORT"], 12345);
        assert_eq!(host["NAME"], NAME);
        let root = response("/", 12345).unwrap();
        assert_eq!(root["CONTENTS"]["avatar"]["CONTENTS"]["change"]["TYPE"], "s");
        assert!(response("/missing", 12345).is_none());
    }
}
