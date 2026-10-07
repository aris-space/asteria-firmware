// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Terminal UI for manually sending messages to the Recovery Board over CAN, and watching what it
//! sends back (status, CATS states, steering positions, occurred events).
//!
//! Run with `recovery-sender [can-interface]` (default: `can0`).
//!
//! Messages are encoded with the same datapoint crates the firmware uses, so they cannot drift.
//! On non-Linux the CAN backend is disabled: messages are dropped and the UI is for show only.

use std::io::{Result as IoResult, Write, stdout};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossterm::cursor::{Hide, MoveTo, Show};
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::style::{Attribute, Color, Print, ResetColor, SetAttribute, SetForegroundColor};
use crossterm::terminal::{
    Clear, ClearType, EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use crossterm::{ExecutableCommand as _, QueueableCommand as _};
use datatypes::status::BoardId;
use dp_recovery_board::{
    DeploymentState, RecoveryBoardStatus, RecoveryPowerConfig, SteeringPositions,
};

/// Interval at which steering targets are repeated while streaming. The firmware watchdog expects
/// regular updates during the steering phase (10 Hz).
const STREAM_INTERVAL: Duration = Duration::from_millis(100);

/// Anything we can put on the bus.
enum Outgoing {
    Recovery(dp_recovery_board::Message),
    System(dp_system_management::Message),
}

/// State written by the CAN threads and read by the UI.
#[derive(Default)]
struct Shared {
    status: String,
    board_status: Option<RecoveryBoardStatus>,
    positions: Option<SteeringPositions>,
    cats_separation: Option<DeploymentState>,
    cats_deployment: Option<DeploymentState>,
    separation_occurred: u32,
    deployment_occurred: u32,
    sent: u32,
}

#[derive(Clone, Copy, PartialEq)]
enum Row {
    PowerSeparation,
    PowerDeployment,
    PowerSteering,
    SendPower,
    Left,
    Right,
    SendSteering,
    SeparationTrigger,
    DeploymentTrigger,
    Reset,
}

const ROWS: [Row; 10] = [
    Row::PowerSeparation,
    Row::PowerDeployment,
    Row::PowerSteering,
    Row::SendPower,
    Row::Left,
    Row::Right,
    Row::SendSteering,
    Row::SeparationTrigger,
    Row::DeploymentTrigger,
    Row::Reset,
];

impl Row {
    /// Triggers actuate hardware, so they need a second Enter.
    fn needs_confirm(self) -> bool {
        matches!(
            self,
            Row::SeparationTrigger | Row::DeploymentTrigger | Row::Reset
        )
    }
}

/// What the user is composing.
struct Form {
    power: RecoveryPowerConfig,
    steering: SteeringPositions,
    selected: usize,
    /// Row waiting for its confirming second Enter.
    confirming: Option<Row>,
    streaming: bool,
    /// What was last sent, to show which sends are still to do.
    sent_power: Option<RecoveryPowerConfig>,
    sent_steering: Option<SteeringPositions>,
}

impl Form {
    fn power_pending(&self) -> bool {
        self.sent_power.as_ref() != Some(&self.power)
    }

    fn steering_pending(&self) -> bool {
        self.sent_steering.as_ref() != Some(&self.steering)
    }

    fn row(&self) -> Row {
        ROWS[self.selected]
    }

    /// Left/right/space on the selected row.
    fn adjust(&mut self, delta: i32) {
        match self.row() {
            Row::PowerSeparation => self.power.separation_enabled ^= true,
            Row::PowerDeployment => self.power.deployment_enabled ^= true,
            Row::PowerSteering => self.power.steering_enabled ^= true,
            Row::Left => self.steering.left_pos = self.steering.left_pos.saturating_add(delta),
            Row::Right => self.steering.right_pos = self.steering.right_pos.saturating_add(delta),
            _ => {}
        }
    }

    fn zero(&mut self) {
        match self.row() {
            Row::Left => self.steering.left_pos = 0,
            Row::Right => self.steering.right_pos = 0,
            _ => {}
        }
    }

    /// Enter on the selected row: returns the message to send, if any.
    fn activate(&mut self, status: &mut String) -> Option<Outgoing> {
        let row = self.row();
        if row.needs_confirm() && self.confirming != Some(row) {
            self.confirming = Some(row);
            *status = "press Enter again to send".into();
            return None;
        }
        self.confirming = None;
        match row {
            Row::SendPower => {
                self.sent_power = Some(self.power.clone());
                Some(Outgoing::Recovery(
                    dp_recovery_board::Message::RecoveryPowerConfig(self.power.clone()),
                ))
            }
            Row::SendSteering => Some(self.steering_message()),
            Row::SeparationTrigger => Some(Outgoing::Recovery(
                dp_recovery_board::Message::SeparationTrigger,
            )),
            Row::DeploymentTrigger => Some(Outgoing::Recovery(
                dp_recovery_board::Message::DeploymentTrigger,
            )),
            Row::Reset => Some(Outgoing::System(
                dp_system_management::Message::ResetSpecific(BoardId::RecoveryBoard),
            )),
            _ => {
                self.adjust(0);
                None
            }
        }
    }

    fn steering_message(&mut self) -> Outgoing {
        self.sent_steering = Some(self.steering.clone());
        Outgoing::Recovery(dp_recovery_board::Message::SteeringTargetPositions(
            self.steering.clone(),
        ))
    }
}

/// Spawns the CAN receive and transmit threads and returns the transmit sender.
///
/// On non-Linux the socket is replaced by a drain so the UI still runs.
fn spawn_can_threads(shared: Arc<Mutex<Shared>>, interface: &str) -> Sender<Outgoing> {
    let (tx, rx) = std::sync::mpsc::channel::<Outgoing>();

    #[cfg(target_os = "linux")]
    {
        use data_core::can::hal::{CanDecode as _, CanEncode as _};
        use embedded_can::Frame as _;
        use socketcan::{BlockingCan as _, Socket as _};

        let sock_rx = socketcan::CanFdSocket::open(interface).expect("open CAN socket");
        let sock_tx = {
            use std::os::fd::AsFd as _;
            socketcan::CanFdSocket::from(sock_rx.as_fd().try_clone_to_owned().expect("dup fd"))
        };

        let shared_rx = Arc::clone(&shared);
        std::thread::spawn(move || {
            let mut sock = sock_rx;
            loop {
                let frame = match sock.receive() {
                    Ok(f) => f,
                    Err(e) => {
                        shared_rx.lock().expect("lock").status =
                            format!("CAN receive error: {e:?}");
                        std::thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                };
                let id = match frame.id() {
                    embedded_can::Id::Standard(id) => id,
                    embedded_can::Id::Extended(e) => e.standard_id(),
                };
                // Other boards' traffic is normal, skip whatever isn't a recovery board message.
                let Ok(msg) = dp_recovery_board::Message::from_parts(id, frame.data()) else {
                    continue;
                };
                let mut st = shared_rx.lock().expect("lock");
                match msg {
                    dp_recovery_board::Message::BoardStatus(s) => st.board_status = Some(s),
                    dp_recovery_board::Message::SteeringActualPositions(p) => {
                        st.positions = Some(p)
                    }
                    dp_recovery_board::Message::CATSSeparationState(s) => {
                        st.cats_separation = Some(s)
                    }
                    dp_recovery_board::Message::CATSDeploymentState(s) => {
                        st.cats_deployment = Some(s)
                    }
                    dp_recovery_board::Message::SeparationOccurred => st.separation_occurred += 1,
                    dp_recovery_board::Message::DeploymentOccurred => st.deployment_occurred += 1,
                    _ => {}
                }
            }
        });

        std::thread::spawn(move || {
            let mut sock = sock_tx;
            for msg in rx {
                let mut buf = [0u8; 64];
                let encoded = match &msg {
                    Outgoing::Recovery(m) => m.encode_into(&mut buf).map_err(|e| format!("{e:?}")),
                    Outgoing::System(m) => m.encode_into(&mut buf).map_err(|e| format!("{e:?}")),
                };
                let (id, len) = match encoded {
                    Ok(x) => x,
                    Err(e) => {
                        shared.lock().expect("lock").status = format!("encode error: {e}");
                        continue;
                    }
                };
                let Some(frame) = socketcan::CanAnyFrame::new(id, &buf[..len as usize]) else {
                    continue;
                };
                let mut st = shared.lock().expect("lock");
                match sock.transmit(&frame) {
                    Ok(()) => {
                        st.sent += 1;
                        st.status = format!("sent id 0x{:X}, {len} bytes", id.as_raw());
                    }
                    Err(e) => st.status = format!("CAN transmit error: {e:?}"),
                }
            }
        });
    }

    #[cfg(not(target_os = "linux"))]
    {
        let _ = interface;
        shared.lock().expect("lock").status =
            "no CAN backend on this OS, messages are dropped".into();
        std::thread::spawn(move || for _ in rx {});
    }

    tx
}

fn put(out: &mut impl Write, col: u16, row: u16, text: &str) -> IoResult<()> {
    out.queue(MoveTo(col, row))?.queue(Print(text))?;
    Ok(())
}

fn on_off(b: bool) -> &'static str {
    if b { "ON " } else { "off" }
}

/// Draws a thin box with a title in the top border.
fn draw_box(out: &mut impl Write, x: u16, y: u16, w: u16, h: u16, title: &str) -> IoResult<()> {
    let inner = "─".repeat(usize::from(w) - 2);
    put(out, x, y, &format!("┌{inner}┐"))?;
    put(out, x + 2, y, title)?;
    for row in 1..h - 1 {
        put(out, x, y + row, "│")?;
        put(out, x + w - 1, y + row, "│")?;
    }
    put(out, x, y + h - 1, &format!("└{inner}┘"))
}

fn show<T: std::fmt::Debug>(o: Option<T>) -> String {
    o.map_or_else(|| "-".to_string(), |v| format!("{v:?}"))
}

fn pair<T: std::fmt::Debug>(a: Option<T>, b: Option<T>) -> String {
    format!("{} / {}", show(a), show(b))
}

/// Formats microseconds as h:mm:ss.
fn uptime(micros: u64) -> String {
    let secs = micros / 1_000_000;
    format!("{}:{:02}:{:02}", secs / 3600, secs / 60 % 60, secs % 60)
}

fn render(out: &mut impl Write, form: &Form, shared: &Shared, interface: &str) -> IoResult<()> {
    out.queue(Clear(ClearType::All))?;
    put(
        out,
        0,
        0,
        &format!("Recovery Board on {interface}, sent: {}", shared.sent),
    )?;

    // (text, still to do)
    let streaming = if form.streaming {
        " (streaming 10 Hz)"
    } else {
        ""
    };
    let lines: [(String, bool); 10] = [
        (
            format!(
                "Power separation   [{}]",
                on_off(form.power.separation_enabled)
            ),
            false,
        ),
        (
            format!(
                "Power deployment   [{}]",
                on_off(form.power.deployment_enabled)
            ),
            false,
        ),
        (
            format!(
                "Power steering     [{}]",
                on_off(form.power.steering_enabled)
            ),
            false,
        ),
        ("  -> send power config".into(), form.power_pending()),
        (
            format!("Steering left      {:>7}", form.steering.left_pos),
            false,
        ),
        (
            format!("Steering right     {:>7}", form.steering.right_pos),
            false,
        ),
        (
            format!("  -> send steering target{streaming}"),
            form.steering_pending(),
        ),
        ("SEND separation trigger".into(), false),
        ("SEND deployment trigger".into(), false),
        ("SEND reset recovery board".into(), false),
    ];
    draw_box(out, 0, 1, 42, 13, " Send ")?;
    for (i, (line, pending)) in lines.iter().enumerate() {
        if i == form.selected {
            out.queue(SetAttribute(Attribute::Reverse))?;
        }
        if *pending || form.confirming == Some(ROWS[i]) {
            out.queue(SetForegroundColor(Color::Red))?;
        }
        put(out, 2, 2 + i as u16, &format!("{line:<38}"))?;
        out.queue(SetAttribute(Attribute::Reset))?
            .queue(ResetColor)?;
    }

    let x = 46;
    draw_box(out, 44, 1, 44, 13, " Received ")?;
    let s = shared.board_status.as_ref();
    let rows = [
        format!("CATS separation  {}", show(shared.cats_separation)),
        format!("CATS deployment  {}", show(shared.cats_deployment)),
        format!("Sep. occurred    {}x", shared.separation_occurred),
        format!("Depl. occurred   {}x", shared.deployment_occurred),
        format!(
            "Steering actual  {}",
            shared
                .positions
                .as_ref()
                .map_or("-".into(), |p| format!("{} / {}", p.left_pos, p.right_pos))
        ),
        format!(
            "Sep. actuators   {}",
            pair(s.map(|s| s.sep1_status), s.map(|s| s.sep2_status))
        ),
        format!(
            "Depl. actuators  {}",
            pair(s.map(|s| s.depl1_status), s.map(|s| s.depl2_status))
        ),
        format!(
            "Steering L / R   {}",
            pair(
                s.map(|s| s.steering_left_status),
                s.map(|s| s.steering_right_status)
            )
        ),
        format!(
            "Watchdog         {}",
            show(s.map(|s| s.steering_watchdog_status))
        ),
        format!("Arming           {}", show(s.map(|s| s.arming_state))),
        format!(
            "Uptime           {}",
            s.map_or("-".to_string(), |s| uptime(s.common.micros_since_restart))
        ),
    ];
    for (i, row) in rows.iter().enumerate() {
        put(out, x, 2 + i as u16, row)?;
    }

    let (_, term_rows) = crossterm::terminal::size()?;
    out.queue(SetForegroundColor(Color::Yellow))?;
    put(out, 0, term_rows.saturating_sub(3), &shared.status)?;
    out.queue(ResetColor)?;
    put(
        out,
        0,
        term_rows.saturating_sub(1),
        "up/down select  left/right change (shift: x10)  enter send  0 zero  s stream steering  q quit",
    )?;
    out.flush()
}

/// Restores the terminal on drop, also when unwinding from a panic.
struct TerminalGuard;

impl TerminalGuard {
    fn enter() -> IoResult<Self> {
        enable_raw_mode()?;
        stdout().execute(EnterAlternateScreen)?.execute(Hide)?;
        Ok(Self)
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let _ = stdout()
            .execute(Show)
            .and_then(|o| o.execute(LeaveAlternateScreen));
        let _ = disable_raw_mode();
    }
}

fn main() -> IoResult<()> {
    let interface = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    let shared = Arc::new(Mutex::new(Shared::default()));
    let can_tx = spawn_can_threads(Arc::clone(&shared), &interface);

    let mut form = Form {
        power: RecoveryPowerConfig::default(),
        steering: SteeringPositions::default(),
        selected: 0,
        confirming: None,
        streaming: false,
        sent_power: None,
        sent_steering: None,
    };
    let _guard = TerminalGuard::enter()?;
    let mut out = stdout();
    let mut last_stream = Instant::now();

    loop {
        render(&mut out, &form, &shared.lock().expect("lock"), &interface)?;

        if event::poll(Duration::from_millis(50))?
            && let Event::Key(key) = event::read()?
            && key.kind == KeyEventKind::Press
        {
            let step = if key.modifiers.contains(KeyModifiers::SHIFT) {
                1000
            } else {
                100
            };
            let mut msg = None;
            let confirming_before = form.confirming;
            match key.code {
                KeyCode::Char('q') | KeyCode::Esc => break,
                KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => break,
                KeyCode::Up => form.selected = form.selected.saturating_sub(1),
                KeyCode::Down => form.selected = (form.selected + 1).min(ROWS.len() - 1),
                KeyCode::Left => form.adjust(-step),
                KeyCode::Right => form.adjust(step),
                KeyCode::Char('0') => form.zero(),
                KeyCode::Char('s') => form.streaming ^= true,
                KeyCode::Enter => {
                    let mut st = shared.lock().expect("lock");
                    msg = form.activate(&mut st.status);
                }
                _ => {}
            }
            // anything but the confirming Enter cancels a pending confirmation
            if key.code != KeyCode::Enter && confirming_before.is_some() {
                form.confirming = None;
            }
            if let Some(msg) = msg {
                let _ = can_tx.send(msg);
            }
        }

        if form.streaming && last_stream.elapsed() >= STREAM_INTERVAL {
            last_stream = Instant::now();
            let _ = can_tx.send(form.steering_message());
        }
    }
    Ok(())
}
