use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::rpc_service::RpcService;
use generic_array::typenum::{U1, U256};
use littlefs2::fs::{Allocation, Filesystem};
use static_cell::StaticCell;
use w25q256jv::LittlefsAdapter;

use crate::resources::flash::{BoardFlash, FixedHighPin, FlashDevice};

type Adapter = LittlefsAdapter<'static, FlashDevice, FixedHighPin, FixedHighPin, U256, U1>;
type Fs = Filesystem<'static, Adapter>;

pub static FS: RpcService<CriticalSectionRawMutex, Fs, 256> = RpcService::new();

static ADAPTER: StaticCell<Adapter> = StaticCell::new();
static ALLOC: StaticCell<Allocation<Adapter>> = StaticCell::new();

#[embassy_executor::task]
pub async fn task(flash: &'static mut BoardFlash) -> ! {
    let adapter = ADAPTER.init(LittlefsAdapter::<_, _, _, U256, U1>::new(flash));
    let alloc = ALLOC.init(Filesystem::allocate());

    if Filesystem::mount(alloc, adapter).is_err() {
        info!("storage: formatting flash");
        Filesystem::format(adapter).expect("format failed");
    }
    let mut fs = Filesystem::mount(alloc, adapter).expect("mount failed");

    info!("storage: filesystem mounted");
    FS.run(&mut fs).await
}
