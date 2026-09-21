//! VRChat OSCQuery avatar discovery, adapted from Shora's oscavmgr port.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::time::{Duration, Instant};

use mdns_sd::{ServiceDaemon, ServiceEvent};
use super::mystery::OscJsonNode;

const RESCAN_INTERVAL: Duration = Duration::from_secs(15);

pub struct OscJsonBrowser {
    mdns: ServiceDaemon,
    events: mdns_sd::Receiver<ServiceEvent>,
    addr: Option<SocketAddr>,
    last_scan: Instant,
}

impl OscJsonBrowser {
    pub fn new() -> Option<Self> {
        let mdns = ServiceDaemon::new().ok()?;
        let _ = mdns.enable_interface(mdns_sd::IfKind::LoopbackV4);
        let events = match mdns.browse("_oscjson._tcp.local.") {
            Ok(events) => events,
            Err(_) => { let _ = mdns.shutdown(); return None; }
        };
        Some(Self { mdns, events, addr: None, last_scan: Instant::now() - RESCAN_INTERVAL })
    }

    pub fn refresh(&mut self) {
        self.last_scan = Instant::now() - RESCAN_INTERVAL;
    }

    pub fn poll_avatar(&mut self) -> Option<OscJsonNode> {
        if self.last_scan.elapsed() < RESCAN_INTERVAL { return None; }
        self.last_scan = Instant::now();
        for event in self.events.try_iter() {
            if let ServiceEvent::ServiceResolved(info) = event {
                if !info.get_fullname().starts_with("VRChat-Client-") { continue; }
                if let Some(ip) = info.get_addresses().iter().find(|ip| ip.is_ipv4()) {
                    self.addr = Some(SocketAddr::new(ip.to_ip_addr(), info.get_port()));
                }
            }
        }
        // Re-fetch periodically and after avatar changes, retrying failed fetches.
        // Static face.avatar files bypass this browser entirely.
        fetch_avatar(self.addr?)
    }
}

impl Drop for OscJsonBrowser {
    fn drop(&mut self) { let _ = self.mdns.shutdown(); }
}

fn fetch_avatar(addr: SocketAddr) -> Option<OscJsonNode> {
    let timeout = Duration::from_millis(200);
    let deadline = Instant::now() + Duration::from_millis(500);
    let mut stream = TcpStream::connect_timeout(&addr, timeout).ok()?;
    stream.set_read_timeout(Some(timeout)).ok()?;
    stream.set_write_timeout(Some(timeout)).ok()?;
    write!(stream, "GET /avatar HTTP/1.1\r\nHost: {addr}\r\nAccept: application/json\r\nConnection: close\r\n\r\n").ok()?;
    let mut bytes = Vec::new();
    let mut buffer = [0; 8192];
    loop {
        let count = stream.read(&mut buffer).ok()?;
        if count == 0 { break; }
        bytes.extend_from_slice(&buffer[..count]);
        if bytes.len() > 1024 * 1024 || Instant::now() > deadline { return None; }
    }
    let text = std::str::from_utf8(&bytes).ok()?;
    let (headers, body) = text.split_once("\r\n\r\n")?;
    if headers.lines().next()?.split_whitespace().nth(1)? != "200" { return None; }
    let node: OscJsonNode = serde_json::from_str(body).ok()?;
    node.get("parameters")?;
    Some(node)
}
