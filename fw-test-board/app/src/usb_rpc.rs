//! USB postcard-rpc server: transport and endpoint dispatch.

use crate::board::UsbDriver;
use crate::logging::{FsRequest, FsResponse};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embassy_usb::UsbDevice;
use postcard_rpc::define_dispatch;
use postcard_rpc::header::VarHeader;
#[allow(unused_imports)]
use postcard_rpc::server::impls::embassy_usb_v0_5::dispatch_impl::spawn_fn;
use postcard_rpc::server::impls::embassy_usb_v0_5::dispatch_impl::{
    WireRxBuf, WireRxImpl, WireSpawnImpl, WireStorage, WireTxImpl,
};
use postcard_rpc::server::impls::embassy_usb_v0_5::{PacketBuffers, USB_FS_MAX_PACKET_SIZE};
use postcard_rpc::server::{Dispatch, Server};
use static_cell::ConstStaticCell;
use wire_types::*;

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

type UsbRpcRawMutex = CriticalSectionRawMutex;
type AppDriver = UsbDriver;
type AppStorage = WireStorage<UsbRpcRawMutex, AppDriver, 1024, 1024, 1024, 1024>;
type BufStorage = PacketBuffers<4096, 4096>;
type AppTx = WireTxImpl<UsbRpcRawMutex, AppDriver>;
type AppRx = WireRxImpl<AppDriver>;
type AppServer = Server<AppTx, AppRx, WireRxBuf, App>;

// ---------------------------------------------------------------------------
// Static storage
// ---------------------------------------------------------------------------

static PBUFS: ConstStaticCell<BufStorage> = ConstStaticCell::new(BufStorage::new());
static STORAGE: AppStorage = AppStorage::new();

static REBOOT_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static PANIC_REQ_CH: Channel<CriticalSectionRawMutex, PanicReq, 1> = Channel::new();

// ---------------------------------------------------------------------------
// Application context
// ---------------------------------------------------------------------------

pub struct AppContext;

// ---------------------------------------------------------------------------
// Dispatcher definition
// ---------------------------------------------------------------------------

define_dispatch! {
    app: App;
    spawn_fn: spawn_fn;
    tx_impl: AppTx;
    spawn_impl: WireSpawnImpl;
    context: AppContext;

    endpoints: {
        list: ENDPOINT_LIST;

        | EndpointTy         | kind     | handler              |
        | ----------         | ----     | -------              |
        | FsInfoEndpoint     | async    | fs_info_handler      |
        | FsListDirEndpoint  | async    | fs_list_dir_handler  |
        | FsStatEndpoint     | async    | fs_stat_handler      |
        | FsReadFileEndpoint | async    | fs_read_file_handler |
        | FsRemoveEndpoint   | async    | fs_remove_handler    |
        | FsEraseStorageEndpoint | async | fs_erase_storage_handler |
        | ResetEndpoint      | async    | reset_handler        |
        | PanicEndpoint      | async    | panic_handler        |
    };
    topics_in: {
        list: TOPICS_IN_LIST;
        | TopicTy | kind | handler |
        | ------- | ---- | ------- |
    };
    topics_out: {
        list: TOPICS_OUT_LIST;
    };
}

// ---------------------------------------------------------------------------
// USB config
// ---------------------------------------------------------------------------

fn usb_config() -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(0x16c0, 0x27DD);
    config.manufacturer = Some("ARIS");
    config.product = Some("fw-test-board");
    config.serial_number = Some("00000001");

    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    config
}

// ---------------------------------------------------------------------------
// Endpoint handlers
// ---------------------------------------------------------------------------

async fn fs_info_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsInfoReq,
) -> FsInfoResp {
    let FsResponse::Info(resp) = crate::logging::fs_request(FsRequest::Info(req)).await else {
        return FsInfoResp {
            err: FsError::Io,
            epoch: crate::logging::fs_epoch(),
            max_chunk: 0,
            max_dir_entries: 0,
        };
    };
    resp
}

async fn fs_list_dir_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsListDirReq,
) -> FsListDirResp {
    let FsResponse::ListDir(resp) = crate::logging::fs_request(FsRequest::ListDir(req)).await
    else {
        return FsListDirResp {
            err: FsError::Io,
            epoch: crate::logging::fs_epoch(),
            entries: heapless::Vec::new(),
            next_cursor: 0,
        };
    };
    resp
}

async fn fs_stat_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsStatReq,
) -> FsStatResp {
    let FsResponse::Stat(resp) = crate::logging::fs_request(FsRequest::Stat(req)).await else {
        return FsStatResp {
            err: FsError::Io,
            epoch: crate::logging::fs_epoch(),
            kind: FsNodeKind::File,
            size_bytes: 0,
        };
    };
    resp
}

async fn fs_read_file_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsReadFileReq,
) -> FsReadFileResp {
    let offset = req.offset;
    let FsResponse::ReadFile(resp) = crate::logging::fs_request(FsRequest::ReadFile(req)).await
    else {
        return FsReadFileResp {
            err: FsError::Io,
            epoch: crate::logging::fs_epoch(),
            offset,
            data: heapless::Vec::new(),
            done: false,
        };
    };
    resp
}

async fn fs_remove_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsRemoveReq,
) -> FsRemoveResp {
    let FsResponse::Remove(resp) = crate::logging::fs_request(FsRequest::Remove(req)).await else {
        return FsRemoveResp {
            err: FsError::Io,
            epoch: crate::logging::fs_epoch(),
        };
    };
    resp
}

async fn fs_erase_storage_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsEraseStorageReq,
) -> FsEraseStorageResp {
    let FsResponse::EraseStorage(resp) =
        crate::logging::fs_request(FsRequest::EraseStorage(req)).await
    else {
        return FsEraseStorageResp {
            err: FsError::Io,
            epoch: crate::logging::fs_epoch(),
        };
    };

    if resp.err == FsError::Ok {
        embedded_utils::info!("erase_storage complete; board will now restart");
        REBOOT_SIGNAL.signal(());
    }

    resp
}

async fn reset_handler(_context: &mut AppContext, _header: VarHeader, _req: ResetReq) -> ResetResp {
    embedded_utils::info!("reset requested via rpc; board will now restart");
    REBOOT_SIGNAL.signal(());
    ResetResp { ok: true }
}

async fn panic_handler(_context: &mut AppContext, _header: VarHeader, req: PanicReq) -> PanicResp {
    let accepted = PANIC_REQ_CH.try_send(req).is_ok();
    if accepted {
        embedded_utils::fmt::warn!("panic requested via rpc; board will now panic");
    } else {
        embedded_utils::fmt::warn!("panic requested via rpc but panic queue was full");
    }
    PanicResp { accepted }
}

// ---------------------------------------------------------------------------
// USB tasks
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn usb_device_task(mut usb: UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

#[embassy_executor::task]
pub async fn reboot_task() -> ! {
    REBOOT_SIGNAL.wait().await;
    Timer::after_millis(500).await;
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
pub async fn panic_task() -> ! {
    let req = PANIC_REQ_CH.receive().await;
    Timer::after_millis(100).await;
    panic!("{}", req.message.as_str());
}

#[embassy_executor::task]
pub async fn usb_rpc_task() {
    let usb_driver: AppDriver = {
        let mut board = crate::board::BOARD
            .try_get()
            .expect("BOARD not initialized")
            .try_lock()
            .expect("BOARD mutex busy while taking usb driver");

        board.usb_driver.take().expect("usb driver not available")
    };

    let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };
    spawner.spawn(usb_rpc_task_inner(usb_driver, spawner).expect("usb_rpc_task_inner token"));
}

#[embassy_executor::task]
async fn usb_rpc_task_inner(usb_driver: AppDriver, spawner: embassy_executor::Spawner) {
    let pbufs = PBUFS.take();
    let config = usb_config();

    let (device, tx_impl, rx_impl) = STORAGE.init(
        usb_driver,
        config,
        pbufs.tx_buf.as_mut_slice(),
        USB_FS_MAX_PACKET_SIZE,
    );
    tx_impl.set_timeout_ms_per_frame(8).await;

    let context = AppContext;
    let dispatcher = App::new(context, spawner.into());
    let vkk = dispatcher.min_key_len();

    let mut server: AppServer = Server::new(
        tx_impl,
        rx_impl,
        pbufs.rx_buf.as_mut_slice(),
        dispatcher,
        vkk,
    );

    spawner.spawn(usb_device_task(device).expect("usb_device_task token"));
    spawner.spawn(reboot_task().expect("reboot_task token"));
    spawner.spawn(panic_task().expect("panic_task token"));

    loop {
        let _ = server.run().await;
    }
}
