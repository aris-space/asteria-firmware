use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use defmt_brtt::DefmtConsumer;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::rpc_service::RpcService;
use generic_array::typenum::{U1, U256};
use littlefs2::fs::{Allocation, Filesystem};
use littlefs2::path;
use littlefs2::path::PathBuf;
use static_cell::StaticCell;
use w25q256jv::LittlefsAdapter;

use crate::resources::flash::{BoardFlash, FixedHighPin, FlashDevice};

type Adapter = LittlefsAdapter<'static, FlashDevice, FixedHighPin, FixedHighPin, U256, U1>;
pub(crate) type Fs = Filesystem<'static, Adapter>;

pub static FS: RpcService<CriticalSectionRawMutex, Fs, 512> = RpcService::new();

use embassy_sync::signal::Signal;

static SESSION_INDEX: AtomicU32 = AtomicU32::new(u32::MAX);
pub static READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn session_idx() -> Option<u32> {
    match SESSION_INDEX.load(Ordering::Relaxed) {
        u32::MAX => None,
        idx => Some(idx),
    }
}

pub fn workdir_path(filename: &str) -> Option<PathBuf> {
    let mut s = heapless::String::<64>::new();
    write!(s, "/log_{}/{}", session_idx()?, filename).ok()?;
    PathBuf::try_from(s.as_bytes()).ok()
}

static ADAPTER: StaticCell<Adapter> = StaticCell::new();
static ALLOC: StaticCell<Allocation<Adapter>> = StaticCell::new();

fn prepare_session(fs: &Filesystem<'_, Adapter>) -> Option<u32> {
    let mut max_index: Option<u32> = None;

    let _ = fs.read_dir_and_then(path!("/"), |dir| {
        for entry in dir {
            let entry = entry?;
            let name = entry.file_name().as_str_ref_with_trailing_nul();
            let name = name.trim_end_matches('\0');
            if let Some(suffix) = name.strip_prefix("log_")
                && let Ok(n) = suffix.parse::<u32>()
            {
                max_index = Some(max_index.map_or(n, |m| m.max(n)));
            }
        }
        Ok(())
    });

    let next = max_index.map_or(0, |n| n + 1);
    let mut dir_name = heapless::String::<32>::new();
    write!(dir_name, "/log_{}", next).ok()?;

    fs.create_dir(&PathBuf::try_from(dir_name.as_bytes()).ok()?)
        .ok()?;

    let mut info_path = heapless::String::<64>::new();
    write!(info_path, "{}/build_info.txt", dir_name.as_str()).ok()?;

    let _ = fs.create_file_and_then(&PathBuf::try_from(info_path.as_bytes()).ok()?, |file| {
        use crate::built;
        let mut buf = heapless::String::<512>::new();
        let _ = write!(
            buf,
            "pkg={}\nprofile={}\ntarget={}\ngit={}\ndirty={}\nfeatures={}\n",
            built::PKG_NAME,
            built::PROFILE,
            built::TARGET,
            built::GIT_COMMIT_HASH_SHORT.unwrap_or("none"),
            match built::GIT_DIRTY {
                Some(true) => "true",
                Some(false) => "false",
                None => "none",
            },
            built::FEATURES_LOWERCASE_STR,
        );
        file.write(buf.as_bytes())?;
        Ok(())
    });

    info!("storage: session {}", dir_name.as_str());
    Some(next)
}

const STAGING_SIZE: usize = 512;

struct DefmtStaging {
    buf: [u8; STAGING_SIZE],
    len: usize,
}

impl DefmtStaging {
    fn new() -> Self {
        Self {
            buf: [0u8; STAGING_SIZE],
            len: 0,
        }
    }

    fn stage(&mut self, fs: &Fs, path: &PathBuf, data: &[u8]) {
        if self.len + data.len() > STAGING_SIZE {
            self.flush(fs, path);
        }
        let n = data.len().min(STAGING_SIZE - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&data[..n]);
        self.len += n;
    }

    fn flush(&mut self, fs: &Fs, path: &PathBuf) {
        if self.len == 0 {
            return;
        }
        let _ = fs.open_file_with_options_and_then(
            |o| o.append(true).create(true),
            path,
            |file| file.write(&self.buf[..self.len]),
        );
        self.len = 0;
    }

    fn drain(&mut self, fs: &Fs, path: &PathBuf, consumer: &mut DefmtConsumer) {
        while let Ok(grant) = consumer.read() {
            let data = grant.buf();
            let dlen = data.len();
            self.stage(fs, path, data);
            grant.release(dlen);
        }
        self.flush(fs, path);
    }
}

#[embassy_executor::task]
pub async fn task(flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
    let adapter = ADAPTER.init(LittlefsAdapter::<_, _, _, U256, U1>::new(flash));
    let alloc = ALLOC.init(Filesystem::allocate());

    // Mount, format + retry on failure. Successful mount is dropped due to
    // borrow-checker constraints (can't keep Ok(fs) while formatting in Err arm),
    // then remounted below.
    if Filesystem::mount(alloc, adapter).is_err() {
        info!("storage: mount failed, formatting");
        let _ = Filesystem::format(adapter);
    }
    let Ok(mut fs) = Filesystem::mount(alloc, adapter) else {
        defmt::error!("storage: mount failed after retries, running without filesystem");
        loop {
            let grant = consumer.wait_for_log().await;
            let len = grant.buf().len();
            grant.release(len);
        }
    };
    info!("storage: filesystem mounted");

    crate::params::load_all(&fs);

    if let Some(idx) = prepare_session(&fs) {
        SESSION_INDEX.store(idx, Ordering::Relaxed);
    }

    READY.signal(());
    info!("storage: ready");

    let defmt_path = workdir_path("defmt.bin")
        .unwrap_or_else(|| PathBuf::try_from(b"/defmt.bin".as_slice()).unwrap());

    let mut staging = DefmtStaging::new();

    loop {
        staging.drain(&fs, &defmt_path, &mut consumer);

        match select(FS.run(&mut fs), consumer.wait_for_log()).await {
            Either::First(_) => unreachable!(),
            Either::Second(grant) => {
                let data = grant.buf();
                let dlen = data.len();
                staging.stage(&fs, &defmt_path, data);
                grant.release(dlen);
            }
        }
    }
}
