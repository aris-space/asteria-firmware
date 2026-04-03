use defmt_brtt::DefmtConsumer;

use crate::resources::flash::BoardFlash;
use crate::storage::{CONFIG_READY, UnavailableStorage};

async fn discard_logs_forever(mut consumer: DefmtConsumer) -> ! {
    loop {
        let grant = consumer.wait_for_log().await;
        let len = grant.buf().len();
        grant.release(len);
    }
}

async fn load_in_memory_defaults() {
    let mut unavailable = UnavailableStorage;
    crate::params::load_all(&mut unavailable).await;
}

async fn run_with_defaults(consumer: DefmtConsumer, message: &'static str) -> ! {
    load_in_memory_defaults().await;
    CONFIG_READY.signal(());
    defmt::warn!("{}", message);
    discard_logs_forever(consumer).await
}

#[cfg(feature = "storage")]
mod with_storage {
    use embassy_embedded_hal::flash::partition::Partition;
    use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
    use embassy_sync::mutex::Mutex;
    use sequential_storage::Error as SeqError;
    use sequential_storage::cache::NoCache;
    use sequential_storage::map::{Key as SeqKey, MapConfig, MapStorage, SerializationError};
    use sequential_storage::queue::{QueueConfig, QueueStorage};
    use static_cell::StaticCell;
    use w25q256jv::{CAPACITY, SECTOR_SIZE};

    use crate::storage::session::{
        DEFMT_CHUNK_SIZE, LOG_RECORD_BUFFER_SIZE, LogRecord, session_meta,
    };

    use super::super::{
        CONFIG_READY, KeyRead, KeyStorage, LogWriteStatus, STORAGE_KEY_CAPACITY, SaveStatus,
        StorageKey, StorageResult, StorageUnavailable,
    };
    use super::{BoardFlash, DefmtConsumer, run_with_defaults};

    const KV_BUFFER_SIZE: usize = 384;
    const SESSION_NEXT_ID_KEY: StorageKey = StorageKey::new("session.next_id");

    #[derive(Clone, Copy)]
    struct Region {
        offset: u32,
        size: u32,
    }

    impl Region {
        const fn new(offset: u32, size: u32) -> Self {
            Self { offset, size }
        }

        const fn end(self) -> u32 {
            self.offset + self.size
        }
    }

    struct FlashLayout;

    impl FlashLayout {
        const KV: Region = Region::new(0, 512 * 1024);
        const LOG: Region = Region::new(Self::KV.end(), CAPACITY - Self::KV.end());

        const fn is_valid() -> bool {
            Self::KV.size >= (SECTOR_SIZE * 2)
                && Self::LOG.size >= SECTOR_SIZE
                && Self::LOG.end() <= CAPACITY
        }
    }

    type SharedFlash = Mutex<ThreadModeRawMutex, &'static mut BoardFlash>;
    type StoragePartition = Partition<'static, ThreadModeRawMutex, &'static mut BoardFlash>;
    type KvMap = MapStorage<StorageKey, StoragePartition, NoCache>;
    type LogQueue = QueueStorage<StoragePartition, NoCache>;

    static FLASH: StaticCell<SharedFlash> = StaticCell::new();

    impl SeqKey for StorageKey {
        fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
            let len = self.len as usize;
            if buffer.len() < len + 2 {
                return Err(SerializationError::BufferTooSmall);
            }

            buffer[..2].copy_from_slice(&(len as u16).to_le_bytes());
            buffer[2..][..len].copy_from_slice(&self.bytes[..len]);
            Ok(len + 2)
        }

        fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
            let total_len = Self::get_len(buffer)?;
            let len = total_len - 2;

            if buffer.len() < total_len {
                return Err(SerializationError::BufferTooSmall);
            }

            let mut bytes = [0; STORAGE_KEY_CAPACITY];
            bytes[..len].copy_from_slice(&buffer[2..][..len]);

            Ok((
                Self {
                    len: len as u8,
                    bytes,
                },
                total_len,
            ))
        }

        fn get_len(buffer: &[u8]) -> Result<usize, SerializationError> {
            if buffer.len() < 2 {
                return Err(SerializationError::BufferTooSmall);
            }

            let len = u16::from_le_bytes(buffer[..2].try_into().unwrap()) as usize;
            if len > STORAGE_KEY_CAPACITY {
                return Err(SerializationError::InvalidData);
            }

            Ok(len + 2)
        }
    }

    fn partition(flash: &'static SharedFlash, region: Region) -> StoragePartition {
        Partition::new(flash, region.offset, region.size)
    }

    struct KvState {
        store: KvMap,
        buffer: [u8; KV_BUFFER_SIZE],
    }

    impl KvState {
        fn new(flash: &'static SharedFlash) -> Self {
            Self {
                store: MapStorage::new(
                    partition(flash, FlashLayout::KV),
                    const { MapConfig::new(0..FlashLayout::KV.size) },
                    NoCache::new(),
                ),
                buffer: [0; KV_BUFFER_SIZE],
            }
        }

        async fn load_u32(&mut self, key: StorageKey) -> Option<u32> {
            self.store
                .fetch_item::<u32>(&mut self.buffer, &key)
                .await
                .ok()?
        }

        async fn store_u32(&mut self, key: StorageKey, value: u32) -> StorageResult {
            self.store
                .store_item(&mut self.buffer, &key, &value)
                .await
                .map_err(|_| StorageUnavailable)
        }

        async fn read_key(&mut self, key: StorageKey, out: &mut [u8]) -> KeyRead {
            match self.store.fetch_item::<&[u8]>(&mut self.buffer, &key).await {
                Ok(Some(value)) if value.len() <= out.len() => {
                    out[..value.len()].copy_from_slice(value);
                    KeyRead::Found(value.len())
                }
                Ok(Some(_)) => KeyRead::Unavailable,
                Ok(None) => KeyRead::Missing,
                Err(_) => KeyRead::Unavailable,
            }
        }

        async fn write_key(&mut self, key: StorageKey, data: &[u8]) -> StorageResult {
            self.store
                .store_item(&mut self.buffer, &key, &data)
                .await
                .map_err(|_| StorageUnavailable)
        }
    }

    impl KeyStorage for KvState {
        async fn read_key(&mut self, key: StorageKey, out: &mut [u8]) -> KeyRead {
            KvState::read_key(self, key, out).await
        }

        async fn write_key(&mut self, key: StorageKey, data: &[u8]) -> SaveStatus {
            KvState::write_key(self, key, data)
                .await
                .map(|_| SaveStatus::Persisted)
                .unwrap_or(SaveStatus::RuntimeOnly)
        }
    }

    struct LogState {
        store: LogQueue,
        record_buffer: [u8; LOG_RECORD_BUFFER_SIZE],
        current_session_id: Option<u32>,
        full: bool,
        full_warned: bool,
    }

    impl LogState {
        fn new(flash: &'static SharedFlash) -> Self {
            Self {
                store: QueueStorage::new(
                    partition(flash, FlashLayout::LOG),
                    const { QueueConfig::new(0..FlashLayout::LOG.size) },
                    NoCache::new(),
                ),
                record_buffer: [0; LOG_RECORD_BUFFER_SIZE],
                current_session_id: None,
                full: false,
                full_warned: false,
            }
        }

        async fn begin_session(&mut self, session_id: u32) -> LogWriteStatus {
            let status = self
                .append_record(&LogRecord::SessionStart(session_meta(session_id)))
                .await;
            if matches!(status, LogWriteStatus::Stored) {
                self.current_session_id = Some(session_id);
            }

            status
        }

        async fn append_defmt_chunk(&mut self, bytes: &[u8]) -> LogWriteStatus {
            let Some(session_id) = self.current_session_id else {
                return LogWriteStatus::Unavailable;
            };

            self.append_record(&LogRecord::DefmtChunk { session_id, bytes })
                .await
        }

        async fn append_record(&mut self, record: &LogRecord<'_>) -> LogWriteStatus {
            if self.full {
                return LogWriteStatus::Full;
            }

            let len = match postcard::to_slice(record, &mut self.record_buffer) {
                Ok(bytes) => bytes.len(),
                Err(_) => return LogWriteStatus::Unavailable,
            };

            match self.store.push(&self.record_buffer[..len], false).await {
                Ok(()) => LogWriteStatus::Stored,
                Err(SeqError::FullStorage) => {
                    self.full = true;
                    if !self.full_warned {
                        self.full_warned = true;
                        defmt::warn!("storage: log region full, dropping future defmt chunks");
                    }
                    LogWriteStatus::Full
                }
                Err(_) => LogWriteStatus::Unavailable,
            }
        }
    }

    struct StorageState {
        kv: KvState,
        log: LogState,
    }

    impl StorageState {
        fn new(flash: &'static SharedFlash) -> Self {
            Self {
                kv: KvState::new(flash),
                log: LogState::new(flash),
            }
        }

        async fn begin_session(&mut self) -> LogWriteStatus {
            let session_id = self.kv.load_u32(SESSION_NEXT_ID_KEY).await.unwrap_or(0);
            let next_id = session_id.saturating_add(1);

            if self
                .kv
                .store_u32(SESSION_NEXT_ID_KEY, next_id)
                .await
                .is_err()
            {
                return LogWriteStatus::Unavailable;
            }

            self.log.begin_session(session_id).await
        }
    }

    async fn init_state(flash: &'static mut BoardFlash) -> Option<StorageState> {
        if !FlashLayout::is_valid() {
            return None;
        }

        let shared_flash = FLASH.init(Mutex::new(flash));
        Some(StorageState::new(shared_flash))
    }

    pub(crate) async fn run(flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
        let Some(mut state) = init_state(flash).await else {
            run_with_defaults(
                consumer,
                "storage: sequential regions invalid, continuing with in-memory defaults",
            )
            .await
        };

        crate::params::load_all(&mut state.kv).await;

        match state.begin_session().await {
            LogWriteStatus::Stored => {
                defmt::info!("storage: sequential backend initialized");
            }
            LogWriteStatus::Full => {
                defmt::warn!("storage: log region full at startup, continuing without new logs");
            }
            LogWriteStatus::Unavailable => {
                run_with_defaults(
                    consumer,
                    "storage: unavailable, continuing with in-memory defaults",
                )
                .await
            }
        }

        CONFIG_READY.signal(());
        defmt::info!("storage: config initialized");

        loop {
            let grant = consumer.wait_for_log().await;
            let len = grant.buf().len();

            if len > 0 {
                let data = grant.buf();
                for chunk in data.chunks(DEFMT_CHUNK_SIZE) {
                    if !matches!(
                        state.log.append_defmt_chunk(chunk).await,
                        LogWriteStatus::Stored
                    ) {
                        break;
                    }
                }
            }

            grant.release(len);
        }
    }
}

#[cfg(not(feature = "storage"))]
mod without_storage {
    use super::{BoardFlash, DefmtConsumer, run_with_defaults};

    pub(crate) async fn run(_flash: &'static mut BoardFlash, consumer: DefmtConsumer) -> ! {
        run_with_defaults(consumer, "storage: disabled, using in-memory defaults").await
    }
}

#[cfg(feature = "storage")]
pub(crate) use with_storage::run;
#[cfg(not(feature = "storage"))]
pub(crate) use without_storage::run;
