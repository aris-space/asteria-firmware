use defmt_brtt::DefmtConsumer;

use crate::resources::flash::BoardFlash;
use crate::storage;

#[embassy_executor::task]
pub async fn task(flash: &'static mut BoardFlash, consumer: DefmtConsumer) -> ! {
    storage::run(flash, consumer).await
}
