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
type Fs = Filesystem<'static, Adapter>;

pub static FS: RpcService<CriticalSectionRawMutex, Fs, 256> = RpcService::new();

static SESSION_INDEX: AtomicU32 = AtomicU32::new(u32::MAX);

pub fn workdir() -> Option<PathBuf> {
    let idx = SESSION_INDEX.load(Ordering::Relaxed);
    if idx == u32::MAX {
        return None;
    }
    let mut s = heapless::String::<32>::new();
    write!(s, "/log_{}", idx).ok()?;
    PathBuf::try_from(s.as_bytes()).ok()
}

pub fn workdir_path(filename: &str) -> Option<PathBuf> {
    let dir = workdir()?;
    let mut s = heapless::String::<64>::new();
    write!(s, "{}/{}", dir.as_str_ref_with_trailing_nul().trim_end_matches('\0'), filename).ok()?;
    PathBuf::try_from(s.as_bytes()).ok()
}

static ADAPTER: StaticCell<Adapter> = StaticCell::new();
static ALLOC: StaticCell<Allocation<Adapter>> = StaticCell::new();

fn prepare_session(fs: &Filesystem<'_, Adapter>) -> u32 {
    let mut max_index: Option<u32> = None;

    let _ = fs.read_dir_and_then(path!("/"), |dir| {
        for entry in dir {
            let entry = entry?;
            let name = entry.file_name().as_str_ref_with_trailing_nul();
            let name = name.trim_end_matches('\0');
            if let Some(suffix) = name.strip_prefix("log_") {
                if let Ok(n) = suffix.parse::<u32>() {
                    max_index = Some(match max_index {
                        Some(m) => m.max(n),
                        None => n,
                    });
                }
            }
        }
        Ok(())
    });

    let next = max_index.map(|n| n + 1).unwrap_or(0);
    let mut dir_name = heapless::String::<32>::new();
    write!(dir_name, "/log_{}", next).expect("dir name overflow");

    let dir_path = PathBuf::try_from(dir_name.as_bytes()).expect("invalid path");
    fs.create_dir(&dir_path).expect("mkdir failed");

    let mut info_path = heapless::String::<64>::new();
    write!(info_path, "{}/build_info.txt", dir_name.as_str()).expect("path overflow");
    let info_path = PathBuf::try_from(info_path.as_bytes()).expect("invalid path");

    fs.create_file_and_then(&info_path, |file| {
        use crate::built;
        let mut buf = heapless::String::<512>::new();
        let _ = write!(buf, "pkg={}\n", built::PKG_NAME);
        let _ = write!(buf, "profile={}\n", built::PROFILE);
        let _ = write!(buf, "target={}\n", built::TARGET);
        let _ = write!(buf, "git={}\n", built::GIT_COMMIT_HASH_SHORT.unwrap_or("none"));
        let _ = write!(buf, "dirty={}\n", match built::GIT_DIRTY {
            Some(true) => "true",
            Some(false) => "false",
            None => "none",
        });
        let _ = write!(buf, "features={}\n", built::FEATURES_LOWERCASE_STR);
        file.write(buf.as_bytes())?;
        Ok(())
    })
    .expect("write build_info failed");

    info!("storage: session {}", dir_name.as_str());
    next
}

fn drain_defmt(fs: &Fs, defmt_path: &PathBuf, consumer: &mut DefmtConsumer) {
    while let Ok(grant) = consumer.read() {
        let len = grant.buf().len();
        {
            let data = grant.buf();
            let _ = fs.open_file_with_options_and_then(
                |o| o.append(true).create(true),
                defmt_path,
                |file| Ok(file.write(data)?),
            );
        }
        grant.release(len);
    }
}

#[embassy_executor::task]
pub async fn task(flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
    let adapter = ADAPTER.init(LittlefsAdapter::<_, _, _, U256, U1>::new(flash));
    let alloc = ALLOC.init(Filesystem::allocate());

    // Try mount to check if we need to format. The successful Filesystem is
    // dropped here due to borrow-checker constraints, then remounted below.
    let needs_format = Filesystem::mount(alloc, adapter).is_err();
    if needs_format {
        info!("storage: formatting flash");
        Filesystem::format(adapter).expect("format failed");
    }
    let mut fs = Filesystem::mount(alloc, adapter).expect("mount failed");
    info!("storage: filesystem mounted");

    let session_idx = prepare_session(&fs);
    SESSION_INDEX.store(session_idx, Ordering::Relaxed);

    let defmt_path = workdir_path("defmt.bin").expect("workdir path failed");

    loop {
        drain_defmt(&fs, &defmt_path, &mut consumer);

        match select(FS.run(&mut fs), consumer.wait_for_log()).await {
            Either::First(_) => unreachable!(),
            Either::Second(grant) => {
                let len = grant.buf().len();
                {
                    let data = grant.buf();
                    let _ = fs.open_file_with_options_and_then(
                        |o| o.append(true).create(true),
                        &defmt_path,
                        |file| Ok(file.write(data)?),
                    );
                }
                grant.release(len);
            }
        }
    }
}
