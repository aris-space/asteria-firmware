//! The console's terminal: an `embedded-io-async` adapter over the CDC class
//! for noline, and output helpers that translate newlines for serial terminals.

use core::fmt::{self, Write as _};

use embedded_io_async_06::{ErrorType, Read, Write};
use heapless::String;

use super::Class;

/// Wrap a string literal in an ANSI SGR colour, resetting after. The result is
/// itself a literal, so it works anywhere a `&str` is expected (e.g. a `say`
/// argument).
macro_rules! paint {
    (red, $s:literal) => {
        concat!("\x1b[31m", $s, "\x1b[0m")
    };
    (green, $s:literal) => {
        concat!("\x1b[32m", $s, "\x1b[0m")
    };
    (yellow, $s:literal) => {
        concat!("\x1b[33m", $s, "\x1b[0m")
    };
}

/// `embedded-io-async` adapter over the CDC class so noline can read and write.
/// CDC reads come a USB packet at a time, so a one-packet buffer hands bytes out
/// in whatever chunk sizes the caller asks for; writes go out a packet at a time.
pub struct ConsoleIo<'a> {
    class: &'a mut Class,
    rx: [u8; 64],
    rx_pos: usize,
    rx_len: usize,
}

impl<'a> ConsoleIo<'a> {
    pub fn new(class: &'a mut Class) -> Self {
        Self {
            class,
            rx: [0; 64],
            rx_pos: 0,
            rx_len: 0,
        }
    }
}

#[derive(Debug)]
pub struct IoError;

impl embedded_io_async_06::Error for IoError {
    fn kind(&self) -> embedded_io_async_06::ErrorKind {
        embedded_io_async_06::ErrorKind::Other
    }
}

impl ErrorType for ConsoleIo<'_> {
    type Error = IoError;
}

impl Read for ConsoleIo<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, IoError> {
        // Refill from a fresh packet, skipping any zero-length ones so this
        // never reports a false end-of-stream.
        while self.rx_pos >= self.rx_len {
            self.rx_len = self
                .class
                .read_packet(&mut self.rx)
                .await
                .map_err(|_| IoError)?;
            self.rx_pos = 0;
        }
        let n = buf.len().min(self.rx_len - self.rx_pos);
        buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
        self.rx_pos += n;
        Ok(n)
    }
}

impl Write for ConsoleIo<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, IoError> {
        let n = buf.len().min(64);
        self.class
            .write_packet(&buf[..n])
            .await
            .map_err(|_| IoError)?;
        Ok(n)
    }
}

/// Write `msg` to the host, translating each `\n` to CRLF so a serial terminal
/// returns to column 0. Best-effort: stops on the first write error (e.g. the
/// host disconnected); the read loop detects the disconnect.
pub async fn say(io: &mut ConsoleIo<'_>, msg: &str) {
    for (i, segment) in msg.split('\n').enumerate() {
        if i > 0 && io.write_all(b"\r\n").await.is_err() {
            return;
        }
        if io.write_all(segment.as_bytes()).await.is_err() {
            return;
        }
    }
}

/// [`say`] for a formatted message.
pub async fn sayf(io: &mut ConsoleIo<'_>, args: fmt::Arguments<'_>) {
    let mut msg: String<512> = String::new();
    let _ = msg.write_fmt(args);
    say(io, &msg).await;
}
