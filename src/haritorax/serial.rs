//! Managed serial workers: reconnect, bounded queues, and joined shutdown.
use std::collections::BTreeMap;
use std::sync::{Arc, RwLock, atomic::{AtomicBool, Ordering}};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde::Serialize;
use tokio::sync::mpsc;
use super::{config::{HaritoraXConfig, Model}, gx6::{Decoder, discover_dongle_ports}, HaritoraXEvent};

#[derive(Debug)]
pub struct InputEvent { pub port: String, pub event: HaritoraXEvent }
#[derive(Debug, Clone, Default, Serialize)]
pub struct PortStatus { pub path: String, pub connected: bool, pub error: Option<String> }
type PortStates = Arc<RwLock<BTreeMap<String, PortStatus>>>;

pub struct Acquisition {
    pub events: mpsc::Receiver<InputEvent>,
    tx: mpsc::Sender<InputEvent>,
    states: PortStates,
    stop: Arc<AtomicBool>,
    workers: Vec<JoinHandle<()>>,
    cfg: HaritoraXConfig,
}

impl Acquisition {
    pub fn start(cfg: HaritoraXConfig) -> anyhow::Result<Self> {
        cfg.validate()?;
        let (tx, events) = mpsc::channel(1024);
        let mut me = Self { events, tx, states: Arc::default(), stop: Arc::new(AtomicBool::new(false)), workers: vec![], cfg };
        me.spawn()?;
        Ok(me)
    }
    fn spawn(&mut self) -> anyhow::Result<()> {
        if !self.cfg.enabled { return Ok(()); }
        let mut ports = if self.cfg.ports.is_empty() { discover_dongle_ports() } else { self.cfg.ports.clone() };
        ports.sort(); ports.dedup();
        for path in ports {
            self.states.write().unwrap().insert(path.clone(), PortStatus { path: path.clone(), ..Default::default() });
            let (tx, states, stop, cfg) = (self.tx.clone(), self.states.clone(), self.stop.clone(), self.cfg.clone());
            self.workers.push(std::thread::Builder::new().name("haritorax-serial".into())
                .spawn(move || worker(path, cfg, tx, states, stop))?);
        }
        Ok(())
    }
    pub fn restart(&mut self) -> anyhow::Result<()> {
        anyhow::ensure!(self.cfg.enabled, "HaritoraX input is disabled");
        self.stop_workers();
        while self.events.try_recv().is_ok() {}
        self.states.write().unwrap().clear();
        self.stop = Arc::new(AtomicBool::new(false));
        self.spawn()
    }
    pub fn ports(&self) -> Vec<PortStatus> { self.states.read().unwrap().values().cloned().collect() }
    fn stop_workers(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for worker in &self.workers { worker.thread().unpark(); }
        for worker in self.workers.drain(..) { let _ = worker.join(); }
    }
}
impl Drop for Acquisition { fn drop(&mut self) { self.stop_workers(); } }

fn state(states: &PortStates, path: &str, connected: bool, error: Option<String>) {
    states.write().unwrap().insert(path.into(), PortStatus { path: path.into(), connected, error });
}
fn emit(tx: &mpsc::Sender<InputEvent>, path: &str, events: Vec<HaritoraXEvent>) {
    for event in events {
        // Dropping an overloaded sample is preferable to unbounded latency.
        let _ = tx.try_send(InputEvent { port: path.into(), event });
    }
}
fn worker(path: String, cfg: HaritoraXConfig, tx: mpsc::Sender<InputEvent>, states: PortStates, stop: Arc<AtomicBool>) {
    while !stop.load(Ordering::Relaxed) {
        match serialport::new(&path, cfg.baud_rate).timeout(Duration::from_millis(100)).open() {
            Ok(mut port) => {
                state(&states, &path, true, None);
                let mut decoder = Decoder::default();
                let result = read_port(port.as_mut(), cfg.model == Model::X2, &mut decoder, &tx, &path, &stop);
                emit(&tx, &path, decoder.disconnect());
                let error = result.err().map(|e| e.to_string());
                state(&states, &path, false, error);
            }
            Err(error) => state(&states, &path, false, Some(error.to_string())),
        }
        if !stop.load(Ordering::Relaxed) { std::thread::park_timeout(Duration::from_secs(2)); }
    }
    state(&states, &path, false, None);
}

fn command(port: &mut dyn serialport::SerialPort, text: &str) -> std::io::Result<()> {
    port.write_all(format!("\n{text}\n").as_bytes())
}
fn read_port(port: &mut dyn serialport::SerialPort, is_x2: bool, decoder: &mut Decoder,
    tx: &mpsc::Sender<InputEvent>, path: &str, stop: &AtomicBool) -> std::io::Result<()> {
    for cmd in ["r0:", "r1:", "r:", "o:"] { command(port, cmd)?; }
    let opened = Instant::now();
    let mut heartbeat = Instant::now();
    let mut delayed = false;
    let mut line = Vec::new();
    let mut discard = false;
    let mut buffer = [0; 1024];
    while !stop.load(Ordering::Relaxed) {
        if !delayed && opened.elapsed() >= Duration::from_millis(1500) {
            for cmd in ["i:", "i0:", "i1:", "o0:", "o1:", "v0:", "v1:"] { command(port, cmd)?; }
            // A waking tracker can miss the first identity request. Match the
            // upstream interpreter's delayed retry of the initial batch.
            for cmd in ["r0:", "r1:", "r:", "o:"] { command(port, cmd)?; }
            delayed = true;
        }
        if heartbeat.elapsed() >= Duration::from_secs(2) {
            command(port, "i:")?;
            for id in decoder.missing_assignment_ports() {
                command(port, &format!("r{id}:"))?;
            }
            heartbeat = Instant::now();
        }
        match port.read(&mut buffer) {
            Ok(0) => return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "serial port closed")),
            Ok(len) => for byte in &buffer[..len] {
                if *byte == b'\n' {
                    if !discard {
                        if let Ok(text) = std::str::from_utf8(&line) { emit(tx, path, decoder.line(text, is_x2)); }
                    }
                    line.clear(); discard = false;
                } else if !discard {
                    if line.len() < 8192 { line.push(*byte); } else { line.clear(); discard = true; }
                }
            },
            Err(error) if matches!(error.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock) => {},
            Err(error) => return Err(error),
        }
    }
    Ok(())
}
