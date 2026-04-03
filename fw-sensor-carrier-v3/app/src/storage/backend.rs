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
    use embassy_futures::block_on;
    use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
    use embassy_sync::mutex::Mutex;
    use embedded_storage::nor_flash::{
        ErrorType as BlockingErrorType, NorFlash as BlockingNorFlash,
        ReadNorFlash as BlockingReadNorFlash,
    };
    use embedded_storage_async::nor_flash::{
        MultiwriteNorFlash, NorFlash as AsyncNorFlash, ReadNorFlash as AsyncReadNorFlash,
    };
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

    const KV_REGION_SIZE: u32 = 512 * 1024;
    const LOG_REGION_OFFSET: u32 = KV_REGION_SIZE;
    const LOG_REGION_SIZE: u32 = CAPACITY - LOG_REGION_OFFSET;
    const KV_BUFFER_SIZE: usize = 384;
    const STORAGE_KEY_CAPACITY: usize = 32;
    const SESSION_NEXT_ID_KEY: StorageKey = "session.next_id";

    type SharedFlash = Mutex<ThreadModeRawMutex, BlockingAsyncFlash>;
    type KvPartition = Partition<'static, ThreadModeRawMutex, BlockingAsyncFlash>;
    type LogPartition = Partition<'static, ThreadModeRawMutex, BlockingAsyncFlash>;
    type MapKey = HeaplessString<STORAGE_KEY_CAPACITY>;
    type KvMap = MapStorage<MapKey, KvPartition, NoCache>;
    type LogQueue = QueueStorage<LogPartition, NoCache>;

    static FLASH: StaticCell<SharedFlash> = StaticCell::new();
    static STATE: Mutex<CriticalSectionRawMutex, Option<StorageState>> = Mutex::new(None);

    struct BlockingAsyncFlash {
        flash: &'static mut BoardFlash,
    }

    impl BlockingAsyncFlash {
        fn new(flash: &'static mut BoardFlash) -> Self {
            Self { flash }
        }
    }

    impl BlockingErrorType for BlockingAsyncFlash {
        type Error = <BoardFlash as BlockingErrorType>::Error;
    }

    impl AsyncReadNorFlash for BlockingAsyncFlash {
        const READ_SIZE: usize = <BoardFlash as BlockingReadNorFlash>::READ_SIZE;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            self.flash.blocking_read(offset, bytes)
        }

        fn capacity(&self) -> usize {
            CAPACITY as usize
        }
    }

    impl AsyncNorFlash for BlockingAsyncFlash {
        const WRITE_SIZE: usize = <BoardFlash as BlockingNorFlash>::WRITE_SIZE;
        const ERASE_SIZE: usize = <BoardFlash as BlockingNorFlash>::ERASE_SIZE;

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.flash.blocking_write(offset, bytes)
        }

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.flash.blocking_erase_range(from, to)
        }
    }

    impl MultiwriteNorFlash for BlockingAsyncFlash {}

    struct StorageState {
        kv: KvMap,
        log: LogQueue,
        kv_buffer: [u8; KV_BUFFER_SIZE],
        log_record_buffer: [u8; LOG_RECORD_BUFFER_SIZE],
        current_session_id: Option<u32>,
        log_full: bool,
        log_full_warned: bool,
    }

    impl StorageState {
        fn new(flash: &'static SharedFlash) -> Self {
            let kv = MapStorage::new(
                Partition::new(flash, 0, KV_REGION_SIZE),
                const { MapConfig::new(0..KV_REGION_SIZE) },
                NoCache::new(),
            );
            let log = QueueStorage::new(
                Partition::new(flash, LOG_REGION_OFFSET, LOG_REGION_SIZE),
                const { QueueConfig::new(0..LOG_REGION_SIZE) },
                NoCache::new(),
            );

            Self {
                kv,
                log,
                kv_buffer: [0; KV_BUFFER_SIZE],
                log_record_buffer: [0; LOG_RECORD_BUFFER_SIZE],
                current_session_id: None,
                log_full: false,
                log_full_warned: false,
            }
        }

        async fn load_session_id(&mut self) -> u32 {
            let Some(key) = map_key(SESSION_NEXT_ID_KEY) else {
                return 0;
            };

            match self.kv.fetch_item::<u32>(&mut self.kv_buffer, &key).await {
                Ok(Some(next_id)) => next_id,
                Ok(None) | Err(_) => 0,
            }
        }

        async fn store_session_id(&mut self, next_id: u32) -> bool {
            let Some(key) = map_key(SESSION_NEXT_ID_KEY) else {
                return false;
            };

            self.kv
                .store_item(&mut self.kv_buffer, &key, &next_id)
                .await
                .is_ok()
        }

        async fn begin_session(&mut self) -> LogWriteStatus {
            let session_id = self.load_session_id().await;
            let next_id = session_id.saturating_add(1);

            if !self.store_session_id(next_id).await {
                return LogWriteStatus::Unavailable;
            }

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
            if self.log_full {
                return LogWriteStatus::Full;
            }

            let len = match postcard::to_slice(record, &mut self.log_record_buffer) {
                Ok(bytes) => bytes.len(),
                Err(_) => return LogWriteStatus::Unavailable,
            };

            match self.log.push(&self.log_record_buffer[..len], false).await {
                Ok(()) => LogWriteStatus::Stored,
                Err(SeqError::FullStorage) => {
                    self.log_full = true;
                    if !self.log_full_warned {
                        self.log_full_warned = true;
                        defmt::warn!("storage: log region full, dropping future defmt chunks");
                    }
                    LogWriteStatus::Full
                }
                Err(_) => LogWriteStatus::Unavailable,
            }
        }
    }

    impl KeyStorage for StorageState {
        fn read_key(&mut self, key: StorageKey, out: &mut [u8]) -> KeyRead {
            let Some(key) = map_key(key) else {
                return KeyRead::Unavailable;
            };

            match block_on(self.kv.fetch_item::<&[u8]>(&mut self.kv_buffer, &key)) {
                Ok(Some(value)) if value.len() <= out.len() => {
                    out[..value.len()].copy_from_slice(value);
                    KeyRead::Found(value.len())
                }
                Ok(Some(_)) => KeyRead::Unavailable,
                Ok(None) => KeyRead::Missing,
                Err(_) => KeyRead::Unavailable,
            }
        }

        fn write_key(&mut self, key: StorageKey, data: &[u8]) -> StorageResult {
            let Some(key) = map_key(key) else {
                return Err(StorageUnavailable);
            };

            block_on(self.kv.store_item(&mut self.kv_buffer, &key, &data))
                .map_err(|_| StorageUnavailable)
        }
    }

    fn map_key(key: StorageKey) -> Option<MapKey> {
        let mut storage_key = MapKey::new();
        storage_key.push_str(key).ok()?;
        Some(storage_key)
    }

    async fn init_state(flash: &'static mut BoardFlash) -> Option<StorageState> {
        if KV_REGION_SIZE < (SECTOR_SIZE * 2) || LOG_REGION_SIZE < SECTOR_SIZE {
            return None;
        }

        let shared_flash = FLASH.init(Mutex::new(BlockingAsyncFlash::new(flash)));
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
            .write_key(key, &data[..len])
            .map(|_| SaveStatus::Persisted)
            .unwrap_or(SaveStatus::RuntimeOnly)
    }

    #[embassy_executor::task]
    pub(crate) async fn task(flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
        let Some(mut state) = init_state(flash).await else {
            defmt::warn!("storage: sequential regions invalid, continuing with in-memory defaults");
            let mut unavailable = UnavailableStorage;
            crate::params::load_all(&mut unavailable);
            CONFIG_READY.signal(());
            discard_logs_forever(consumer).await
        };

        crate::params::load_all(&mut state);

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
                let mut unavailable = UnavailableStorage;
                crate::params::load_all(&mut unavailable);
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
                            state.append_defmt_chunk(chunk).await,
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

    pub(crate) async fn persist_key<const N: usize>(
        _key: StorageKey,
        _data: [u8; N],
        _len: usize,
    ) -> SaveStatus {
        SaveStatus::RuntimeOnly
    }

    #[embassy_executor::task]
    pub(crate) async fn task(_flash: &'static mut BoardFlash, consumer: DefmtConsumer) -> ! {
        let mut unavailable = UnavailableStorage;
        crate::params::load_all(&mut unavailable);
        CONFIG_READY.signal(());
        defmt::info!("storage: disabled, using in-memory defaults");
        discard_logs_forever(consumer).await
    }
}

pub(crate) use imp::{persist_key, task};
