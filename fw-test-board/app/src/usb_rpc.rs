//! USB postcard-rpc server: transport and endpoint dispatch.

#![allow(clippy::wildcard_imports)]

use crate::board::UsbDriver;
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

const USB_CONFIG_DESC_CAP: usize = 1024;
const USB_BOS_DESC_CAP: usize = 1024;
const USB_CONTROL_BUF_CAP: usize = 1024;
const USB_MSOS_DESC_CAP: usize = 1024;
const USB_PACKET_TX_BUF_CAP: usize = 4096;
const USB_PACKET_RX_BUF_CAP: usize = 4096;
const PANIC_REQ_CH_CAP: usize = 1;

pub const BOARD_NAME: &str = "test-board";

const USB_VENDOR_ID: u16 = 0x16c0;
const USB_PRODUCT_ID: u16 = 0x27DD;
const USB_MANUFACTURER: &str = "ARIS";
const USB_PRODUCT: &str = BOARD_NAME;
const USB_SERIAL_NUMBER: &str = "00000001";
const USB_DEVICE_CLASS_MISC: u8 = 0xEF;
const USB_DEVICE_SUB_CLASS_COMMON: u8 = 0x02;
const USB_DEVICE_PROTOCOL_IAD: u8 = 0x01;

const REBOOT_DELAY_MS: u64 = 500;
const PANIC_DELAY_MS: u64 = 100;
const USB_TX_TIMEOUT_MS_PER_FRAME: usize = 8;

type AppStorage = WireStorage<
    UsbRpcRawMutex,
    AppDriver,
    USB_CONFIG_DESC_CAP,
    USB_BOS_DESC_CAP,
    USB_CONTROL_BUF_CAP,
    USB_MSOS_DESC_CAP,
>;
type BufStorage = PacketBuffers<USB_PACKET_TX_BUF_CAP, USB_PACKET_RX_BUF_CAP>;
type AppTx = WireTxImpl<UsbRpcRawMutex, AppDriver>;
type AppRx = WireRxImpl<AppDriver>;
type AppServer = Server<AppTx, AppRx, WireRxBuf, App>;

// ---------------------------------------------------------------------------
// Static storage
// ---------------------------------------------------------------------------

static PBUFS: ConstStaticCell<BufStorage> = ConstStaticCell::new(BufStorage::new());
static STORAGE: AppStorage = AppStorage::new();

static REBOOT_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static PANIC_REQ_CH: Channel<CriticalSectionRawMutex, PanicReq, PANIC_REQ_CH_CAP> = Channel::new();

// ---------------------------------------------------------------------------
// Application context
// ---------------------------------------------------------------------------

/// RPC handler context. Kept as a unit struct for now because endpoints are stateless.
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
    let mut config = embassy_usb::Config::new(USB_VENDOR_ID, USB_PRODUCT_ID);
    config.manufacturer = Some(USB_MANUFACTURER);
    config.product = Some(USB_PRODUCT);
    config.serial_number = Some(USB_SERIAL_NUMBER);

    config.device_class = USB_DEVICE_CLASS_MISC;
    config.device_sub_class = USB_DEVICE_SUB_CLASS_COMMON;
    config.device_protocol = USB_DEVICE_PROTOCOL_IAD;
    config.composite_with_iads = true;
    // docs say this should be set even if only sometimes powered,
    // which is the case when board is inside stack.
    config.self_powered = true;

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
    crate::logging::FS_MAILBOX.request(req).await
}

async fn fs_list_dir_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsListDirReq,
) -> FsListDirResp {
    crate::logging::FS_MAILBOX.request(req).await
}

async fn fs_stat_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsStatReq,
) -> FsStatResp {
    crate::logging::FS_MAILBOX.request(req).await
}

async fn fs_read_file_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsReadFileReq,
) -> FsReadFileResp {
    crate::logging::FS_MAILBOX.request(req).await
}

async fn fs_remove_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsRemoveReq,
) -> FsRemoveResp {
    crate::logging::FS_MAILBOX.request(req).await
}

async fn fs_erase_storage_handler(
    _context: &mut AppContext,
    _header: VarHeader,
    req: FsEraseStorageReq,
) -> FsEraseStorageResp {
    let resp = crate::logging::FS_MAILBOX.request(req).await;

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
/// Runs the raw USB device state machine forever.
pub async fn usb_device_task(mut usb: UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

#[embassy_executor::task]
/// Waits for a reboot signal, then resets the MCU after a short grace period.
pub async fn reboot_task() -> ! {
    REBOOT_SIGNAL.wait().await;
    Timer::after_millis(REBOOT_DELAY_MS).await;
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
/// Waits for a panic request and triggers a deliberate panic for testing.
pub async fn panic_task() -> ! {
    let req = PANIC_REQ_CH.receive().await;
    Timer::after_millis(PANIC_DELAY_MS).await;
    panic!("{}", req.message.as_str());
}

#[embassy_executor::task]
/// Starts the USB RPC server on the async executor.
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
/// Initializes USB transport/server internals and serves RPC requests forever.
async fn usb_rpc_task_inner(usb_driver: AppDriver, spawner: embassy_executor::Spawner) {
    let pbufs = PBUFS.take();
    let config = usb_config();

    let (device, tx_impl, rx_impl) = STORAGE.init(
        usb_driver,
        config,
        pbufs.tx_buf.as_mut_slice(),
        USB_FS_MAX_PACKET_SIZE,
    );
    tx_impl
        .set_timeout_ms_per_frame(USB_TX_TIMEOUT_MS_PER_FRAME)
        .await;

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
