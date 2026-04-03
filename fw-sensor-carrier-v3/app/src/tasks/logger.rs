use defmt::{info, warn};
use defmt_brtt::DefmtConsumer;
use littlefs2::path;

use crate::storage;

#[embassy_executor::task]
pub async fn task(mut consumer: DefmtConsumer) -> ! {
    info!("logger: started");

    loop {
        let grant = consumer.wait_for_log().await;
        let data = grant.buf();
        let len = data.len();

        let mut buf = heapless::Vec::<u8, 256>::new();
        if buf.extend_from_slice(data).is_err() {
            warn!("logger: frame too large ({}), dropping", len);
            grant.release(len);
            continue;
        }
        grant.release(len);

        storage::FS
            .call(move |fs| {
                if fs
                    .open_file_with_options_and_then(
                        |o| o.append(true).create(true),
                        path!("/defmt.bin"),
                        |file| Ok(file.write(&buf)?),
                    )
                    .is_err()
                {
                    warn!("logger: write failed");
                }
            })
            .await;
    }
}
