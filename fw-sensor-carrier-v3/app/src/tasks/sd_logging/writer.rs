use embedded_io_async::{Seek, SeekFrom, Write};

// Aligned, whole-block writes let SDMMC use multi-block transfers.
const BLOCK_SIZE: usize = 512;
const BUFFER_SIZE: usize = 8 * BLOCK_SIZE;

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

/// Buffers rows for whole-block SD writes.
/// Flush rewinds the partial block so later writes stay block-aligned.
pub struct CsvLog<'buf, F> {
    pub name: &'static str,
    file: F,
    buffer: &'buf mut LogBuffer,
    used: usize,
    rows: u32,
}

impl<'buf, F: Write + Seek> CsvLog<'buf, F> {
    pub fn new(name: &'static str, header: &str, file: F, buffer: &'buf mut LogBuffer) -> Self {
        buffer.bytes[..header.len()].copy_from_slice(header.as_bytes());
        Self {
            name,
            file,
            buffer,
            used: header.len(),
            rows: 0,
        }
    }

    pub async fn append(&mut self, row: &[u8]) -> Result<(), F::Error> {
        if self.used + row.len() > BUFFER_SIZE {
            self.write_blocks().await?;
        }
        self.buffer.bytes[self.used..][..row.len()].copy_from_slice(row);
        self.used += row.len();
        self.rows += 1;
        Ok(())
    }

    async fn write_blocks(&mut self) -> Result<(), F::Error> {
        let whole = self.used / BLOCK_SIZE * BLOCK_SIZE;
        self.file.write_all(&self.buffer.bytes[..whole]).await?;
        self.buffer.bytes.copy_within(whole..self.used, 0);
        self.used -= whole;
        Ok(())
    }

    /// Writes buffered rows and returns the count since the last flush.
    pub async fn flush(&mut self) -> Result<u32, F::Error> {
        self.write_blocks().await?;
        if self.used > 0 {
            self.file.write_all(&self.buffer.bytes[..self.used]).await?;
            self.file
                .seek(SeekFrom::Current(-(self.used as i64)))
                .await?;
        }
        self.file.flush().await?;
        Ok(core::mem::take(&mut self.rows))
    }
}
