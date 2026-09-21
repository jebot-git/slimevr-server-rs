//! Shora's ratatui frontend, expanded for the integrated native server.
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread::JoinHandle;
use std::time::Duration;
use crossterm::{event::{self, Event, KeyCode, KeyEventKind, KeyModifiers}, execute, terminal::{enable_raw_mode, disable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen}};
use ratatui::{Terminal, backend::CrosstermBackend, layout::{Constraint, Direction, Layout}, style::{Color, Modifier, Style}, widgets::{Block, Borders, Paragraph, Row, Table}, Frame};
use serde::Deserialize;
use crate::{control::{Command, CommandSender}, status::{StatusHandle, StatusSnapshot}};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum UiMode { #[default] Headless, Tui }

pub struct TuiHandle { stop: Arc<AtomicBool>, worker: Option<JoinHandle<anyhow::Result<()>>> }
impl TuiHandle {
    pub fn start(status: StatusHandle, commands: CommandSender) -> anyhow::Result<Self> {
        use std::io::IsTerminal;
        anyhow::ensure!(std::io::stdin().is_terminal() && std::io::stdout().is_terminal(), "--ui tui needs an interactive terminal");
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        let worker = std::thread::Builder::new().name("shora-tui".into()).spawn(move || {
            let _screen = Screen::enter()?;
            let mut terminal = Terminal::new(CrosstermBackend::new(std::io::stdout()))?;
            loop {
                terminal.draw(|frame| draw(frame, &status.read().unwrap()))?;
                if flag.load(Ordering::Relaxed) { break; }
                if event::poll(Duration::from_millis(100))? {
                    if let Event::Key(key) = event::read()? {
                        if key.kind != KeyEventKind::Press { continue; }
                        let command = match key.code {
                            KeyCode::Char('q') | KeyCode::Esc => Some(Command::Shutdown),
                            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => Some(Command::Shutdown),
                            KeyCode::Char('y') => Some(Command::YawReset),
                            KeyCode::Char('f') => Some(Command::FullReset),
                            KeyCode::Char('m') => Some(Command::MountingReset),
                            KeyCode::Char('p') => Some(Command::PauseTracking),
                            KeyCode::Char('r') => Some(Command::Restart),
                            KeyCode::Char('s') => Some(Command::ShutdownTrackers),
                            _ => None,
                        };
                        if let Some(command) = command { let _ = commands.try_send(command.into()); }
                    }
                }
            }
            Ok(())
        })?;
        Ok(Self { stop, worker: Some(worker) })
    }
    pub fn check_health(&mut self) -> anyhow::Result<()> {
        if self.worker.as_ref().is_some_and(|w| w.is_finished()) {
            self.worker.take().unwrap().join().map_err(|_| anyhow::anyhow!("TUI panicked"))??;
            anyhow::bail!("TUI stopped unexpectedly");
        }
        Ok(())
    }
}
impl Drop for TuiHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() { let _ = worker.join(); }
    }
}
struct Screen;
impl Screen {
    fn enter() -> anyhow::Result<Self> {
        enable_raw_mode()?;
        let guard = Self;
        execute!(std::io::stdout(), EnterAlternateScreen)?;
        Ok(guard)
    }
}
impl Drop for Screen {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = execute!(std::io::stdout(), LeaveAlternateScreen, crossterm::cursor::Show);
    }
}

fn draw(frame: &mut Frame<'_>, snapshot: &StatusSnapshot) {
    let areas = Layout::default().direction(Direction::Vertical).constraints([
        Constraint::Length(3), Constraint::Length(5), Constraint::Min(5), Constraint::Length(4), Constraint::Length(3)
    ]).split(frame.area());
    let mode = if snapshot.paused { "PAUSED" } else { "TRACKING" };
    frame.render_widget(Paragraph::new(format!(" Shora · Native tracking hub      {mode}      v{}", snapshot.version))
        .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
        .block(Block::default().borders(Borders::BOTTOM)), areas[0]);
    frame.render_widget(Paragraph::new(format!(
        "{} trackers  ·  {} bones  ·  HMD {}  ·  uptime {}s\nFace: {} → {} ({})   HaritoraX: {}\nUDP :{}   {}",
        snapshot.trackers.len(), snapshot.bone_count, if snapshot.hmd_pose_received { "received" } else { "waiting" }, snapshot.uptime_secs,
        snapshot.face_source, snapshot.face_output, if snapshot.face_active { "active" } else { "waiting / off" },
        if snapshot.haritorax_enabled { "enabled" } else { "disabled" }, snapshot.tracker_port, snapshot.last_action))
        .block(Block::default().borders(Borders::ALL).title(" Server ")), areas[1]);
    let rows = snapshot.trackers.iter().map(|t| Row::new(vec![t.name.clone(),
        t.position.to_string(), if t.active { "active".into() } else { "waiting".into() },
        t.battery_percent.map(|b| format!("{b:.0}%")).unwrap_or_else(|| "—".into()),
        format!("{}ms", t.age_ms), t.source.clone()]));
    let table = Table::new(rows, [Constraint::Percentage(22), Constraint::Length(6), Constraint::Length(9), Constraint::Length(8), Constraint::Length(9), Constraint::Min(12)])
        .header(Row::new(["Tracker", "Body", "State", "Battery", "Seen", "Source"]).style(Style::default().fg(Color::Cyan)))
        .block(Block::default().borders(Borders::ALL).title(" Trackers · SlimeVR + HaritoraX "));
    frame.render_widget(table, areas[2]);
    let ports = if snapshot.ports.is_empty() { "No serial ports open. Enable HaritoraX and press r to rescan.".into() }
        else { snapshot.ports.iter().map(|p| format!("{}: {}", p.path, if p.connected { "connected" } else { p.error.as_deref().unwrap_or("waiting") })).collect::<Vec<_>>().join("\n") };
    frame.render_widget(Paragraph::new(ports).block(Block::default().borders(Borders::ALL).title(" Serial input ")), areas[3]);
    frame.render_widget(Paragraph::new(format!("y yaw · f full · m mounting · p pause · r reconnect/rescan · s trackers off · q quit\nLog: {}", snapshot.log_file.as_deref().unwrap_or("stderr")))
        .style(Style::default().fg(Color::DarkGray)), areas[4]);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn renders_native_status_and_controls() {
        let mut terminal = Terminal::new(ratatui::backend::TestBackend::new(100, 30)).unwrap();
        terminal.draw(|frame| draw(frame, &StatusSnapshot { version: "test".into(), paused: true, ..Default::default() })).unwrap();
        let text: String = terminal.backend().buffer().content.iter().map(|cell| cell.symbol()).collect();
        assert!(text.contains("PAUSED"));
        assert!(text.contains("reconnect/rescan"));
    }
}
