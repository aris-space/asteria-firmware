//! `cal set` and `cal show`.

use core::str::SplitAsciiWhitespace;

use super::io::{ConsoleIo, say, sayf};
use crate::calibration::{self, Calibrations, Correction, baro, dht, gnss, imu, mag};
use crate::sensors::SensorId;
use crate::storage::Storage;

pub async fn command(
    io: &mut ConsoleIo<'_>,
    args: &mut SplitAsciiWhitespace<'_>,
    storage: &Storage,
) {
    match (args.next(), args.next()) {
        (Some("set"), Some(sensor)) => match calibration::set(storage, sensor, args).await {
            Ok(()) => {
                sayf(
                    io,
                    format_args!(paint!(green, "stored {}; reset to apply\n"), sensor),
                )
                .await
            }
            Err(reason) => {
                sayf(
                    io,
                    format_args!(paint!(red, "{} not stored: {}\n"), sensor, reason),
                )
                .await
            }
        },
        (Some("show"), None) => {
            show_cals(io, storage, &imu::CAL).await;
            show_cals(io, storage, &mag::CAL).await;
            show_cals(io, storage, &gnss::CAL).await;
            show_cals(io, storage, &baro::CAL).await;
            show_cals(io, storage, &dht::CAL).await;
        }
        _ => {
            say(
                io,
                paint!(red, "usage: cal <set <id> <key>=<value>...|show>\n"),
            )
            .await
        }
    }
}

/// Prints each applied calibration as the `cal set` line that stores it, and
/// any different one waiting in flash for a reset.
async fn show_cals<Id: SensorId, C: Correction, const N: usize>(
    io: &mut ConsoleIo<'_>,
    storage: &Storage,
    cals: &Calibrations<Id, C, N>,
) {
    for id in cals.ids() {
        let applied = cals.applied(id);
        sayf(io, format_args!("cal set {} {}\n", id.name(), applied)).await;
        if let Some(stored) = cals.stored(storage, id).await.filter(|s| *s != applied) {
            sayf(
                io,
                format_args!(
                    paint!(yellow, "  in flash, applies after reset: cal set {} {}\n"),
                    id.name(),
                    stored
                ),
            )
            .await;
        }
    }
}
