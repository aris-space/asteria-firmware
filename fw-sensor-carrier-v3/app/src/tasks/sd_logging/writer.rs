use embedded_io_async::Write;

use super::{SdDir, SdError, SdFile};

// Aligned, whole-block writes let SDMMC use multi-block transfers.
const BLOCK_SIZE: usize = 512;
const BUFFER_SIZE: usize = 8 * BLOCK_SIZE;
// A new file keeps each one short and bounds what a bad one can lose.
const SEGMENT_SIZE: usize = 32 * 1024 * 1024;

#[repr(align(4))]
pub struct LogBuffer {
    bytes: [u8; BUFFER_SIZE],
}

impl LogBuffer {
    pub const fn new() -> Self {
        Self {
            bytes: [0; BUFFER_SIZE],
        }
    }
}

/// Appends frames to numbered segment files, always in whole blocks.
pub struct SegmentLog<'fs, 'sd, 'buf> {
    dir: SdDir<'fs, 'sd>,
    file: SdFile<'fs, 'sd>,
    segment: u32,
    segment_len: usize,
    buffer: &'buf mut LogBuffer,
    used: usize,
}

impl<'fs, 'sd, 'buf> SegmentLog<'fs, 'sd, 'buf> {
    pub async fn new(
        dir: SdDir<'fs, 'sd>,
        buffer: &'buf mut LogBuffer,
    ) -> Result<Self, SdError<'sd>> {
        // A session resumed after a card error continues with the next free number.
        let mut segment = 1;
        while dir.open_file(&segment_name(segment)).await.is_ok() {
            segment += 1;
        }
        let file = dir.create_file(&segment_name(segment)).await?;
        Ok(Self {
            dir,
            file,
            segment,
            segment_len: 0,
            buffer,
            used: 0,
        })
    }

    pub async fn append(&mut self, frame: &[u8]) -> Result<(), SdError<'sd>> {
        if self.used + frame.len() > BUFFER_SIZE {
            self.write_blocks().await?;
        }
        self.buffer.bytes[self.used..][..frame.len()].copy_from_slice(frame);
        self.used += frame.len();
        Ok(())
    }

    /// Writes whole blocks, keeping the partial block for the next call.
    async fn write_blocks(&mut self) -> Result<(), SdError<'sd>> {
        let whole = self.used / BLOCK_SIZE * BLOCK_SIZE;
        self.file.write_all(&self.buffer.bytes[..whole]).await?;
        self.segment_len += whole;
        self.buffer.bytes.copy_within(whole..self.used, 0);
        self.used -= whole;
        Ok(())
    }

    /// Pads the partial block with zeros (empty frames) and writes everything
    /// out, so a power loss costs at most what came after this call.
    pub async fn flush(&mut self) -> Result<(), SdError<'sd>> {
        if self.used > 0 {
            let padded = self.used.next_multiple_of(BLOCK_SIZE);
            self.buffer.bytes[self.used..padded].fill(0);
            self.used = padded;
            self.write_blocks().await?;
        }
        self.file.flush().await?;
        if self.segment_len >= SEGMENT_SIZE {
            self.segment += 1;
            self.segment_len = 0;
            self.file = self.dir.create_file(&segment_name(self.segment)).await?;
        }
        Ok(())
    }
}

fn segment_name(number: u32) -> heapless::String<12> {
    let mut name = heapless::String::new();
    core::fmt::write(&mut name, format_args!("SEG{number:04}.BIN")).unwrap();
    name
}
