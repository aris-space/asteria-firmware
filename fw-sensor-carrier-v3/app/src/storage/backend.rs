use defmt_brtt::DefmtConsumer;

use crate::resources::flash::BoardFlash;

async fn discard_logs_forever(mut consumer: DefmtConsumer) -> ! {
    loop {
        let grant = consumer.wait_for_log().await;
        let len = grant.buf().len();
        grant.release(len);
    }
}

#[cfg(feature = "storage")]
mod imp {
    use embassy_embedded_hal::flash::partition::Partition;
    use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
    use embassy_sync::mutex::Mutex;
    use heapless::String as HeaplessString;
    use sequential_storage::Error as SeqError;
    use sequential_storage::cache::NoCache;
    use sequential_storage::map::{MapConfig, MapStorage};
    use sequential_storage::queue::{QueueConfig, QueueStorage};
    use static_cell::StaticCell;
    use w25q256jv::{CAPACITY, SECTOR_SIZE};

    use crate::storage::session::{
        DEFMT_CHUNK_SIZE, LOG_RECORD_BUFFER_SIZE, LogRecord, session_meta,
    };

    use super::super::{
        CONFIG_READY, KeyRead, KeyStorage, LogWriteStatus, SaveStatus, StorageKey, StorageResult,
        StorageUnavailable, UnavailableStorage, is_available, set_available,
    };
    use super::{BoardFlash, DefmtConsumer, discard_logs_forever};

    const KV_BUFFER_SIZE: usize = 384;
    const STORAGE_KEY_CAPACITY: usize = 32;
    const SESSION_NEXT_ID_KEY: StorageKey = "session.next_id";

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
    type MapKey = HeaplessString<STORAGE_KEY_CAPACITY>;
    type KvMap = MapStorage<MapKey, StoragePartition, NoCache>;
    type LogQueue = QueueStorage<StoragePartition, NoCache>;

    static FLASH: StaticCell<SharedFlash> = StaticCell::new();
    static STATE: Mutex<CriticalSectionRawMutex, Option<StorageState>> = Mutex::new(None);

    async fn load_in_memory_defaults() {
        let mut unavailable = UnavailableStorage;
        crate::params::load_all(&mut unavailable).await;
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
            let key = storage_key(key)?;
            self.store
                .fetch_item::<u32>(&mut self.buffer, &key)
                .await
                .ok()?
        }

        async fn store_u32(&mut self, key: StorageKey, value: u32) -> StorageResult {
            let key = storage_key(key).ok_or(StorageUnavailable)?;
            self.store
                .store_item(&mut self.buffer, &key, &value)
                .await
                .map_err(|_| StorageUnavailable)
        }

        async fn read_key(&mut self, key: StorageKey, out: &mut [u8]) -> KeyRead {
            let Some(key) = storage_key(key) else {
                return KeyRead::Unavailable;
            };

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
            let key = storage_key(key).ok_or(StorageUnavailable)?;
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

    fn storage_key(key: StorageKey) -> Option<MapKey> {
        let mut storage_key = MapKey::new();
        storage_key.push_str(key).ok()?;
        Some(storage_key)
    }

    async fn init_state(flash: &'static mut BoardFlash) -> Option<StorageState> {
        if !FlashLayout::is_valid() {
            return None;
        }

        let shared_flash = FLASH.init(Mutex::new(flash));
        Some(StorageState::new(shared_flash))
    }

    pub(crate) async fn persist_key<const N: usize>(
        key: StorageKey,
        data: [u8; N],
        len: usize,
    ) -> SaveStatus {
        if !is_available() || len > N {
            return SaveStatus::RuntimeOnly;
        }

        let mut state = STATE.lock().await;
        let Some(state) = state.as_mut() else {
            return SaveStatus::RuntimeOnly;
        };

        state
            .kv
            .write_key(key, &data[..len])
            .await
            .map(|_| SaveStatus::Persisted)
            .unwrap_or(SaveStatus::RuntimeOnly)
    }

    pub(crate) async fn run(flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
        let Some(mut state) = init_state(flash).await else {
            defmt::warn!("storage: sequential regions invalid, continuing with in-memory defaults");
            load_in_memory_defaults().await;
            CONFIG_READY.signal(());
            discard_logs_forever(consumer).await
        };

        crate::params::load_all(&mut state.kv).await;

        match state.begin_session().await {
            LogWriteStatus::Stored => {
                set_available(true);
                defmt::info!("storage: sequential backend initialized");
            }
            LogWriteStatus::Full => {
                set_available(true);
                defmt::warn!("storage: log region full at startup, continuing without new logs");
            }
            LogWriteStatus::Unavailable => {
                defmt::warn!("storage: unavailable, continuing with in-memory defaults");
                load_in_memory_defaults().await;
                CONFIG_READY.signal(());
                discard_logs_forever(consumer).await
            }
        }

        {
            let mut slot = STATE.lock().await;
            *slot = Some(state);
        }

        CONFIG_READY.signal(());
        defmt::info!("storage: config initialized");

        loop {
            let grant = consumer.wait_for_log().await;
            let len = grant.buf().len();

            if len > 0 {
                let data = grant.buf();
                let mut state = STATE.lock().await;
                if let Some(state) = state.as_mut() {
                    for chunk in data.chunks(DEFMT_CHUNK_SIZE) {
                        if !matches!(
                            state.log.append_defmt_chunk(chunk).await,
                            LogWriteStatus::Stored
                        ) {
                            break;
                        }
                    }
                }
            }

            grant.release(len);
        }
    }
}

#[cfg(not(feature = "storage"))]
mod imp {
    use super::{BoardFlash, DefmtConsumer, discard_logs_forever};
    use crate::storage::{CONFIG_READY, SaveStatus, StorageKey, UnavailableStorage};

    async fn load_in_memory_defaults() {
        let mut unavailable = UnavailableStorage;
        crate::params::load_all(&mut unavailable).await;
    }

    pub(crate) async fn persist_key<const N: usize>(
        _key: StorageKey,
        _data: [u8; N],
        _len: usize,
    ) -> SaveStatus {
        SaveStatus::RuntimeOnly
    }

    pub(crate) async fn run(_flash: &'static mut BoardFlash, consumer: DefmtConsumer) -> ! {
        load_in_memory_defaults().await;
        CONFIG_READY.signal(());
        defmt::info!("storage: disabled, using in-memory defaults");
        discard_logs_forever(consumer).await
    }
}

pub(crate) use imp::{persist_key, run};
