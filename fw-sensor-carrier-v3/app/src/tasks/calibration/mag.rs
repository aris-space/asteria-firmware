//! Magnetometer calibration task: on `RUN_MAG_CAL`, fold raw samples
//! from both sensors into per-sensor streaming solvers over a tumble
//! window, fit each independently, and store plausible results.

use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, with_timeout};
use firmware_params::Param;
use magcal::Solver;

use crate::calibration::mag::{MAG_0_PARAM, MAG_1_PARAM, MagCalWire};
use crate::commands::RUN_MAG_CAL;
use crate::params::Access;
use crate::sensors::{MAG_BUS_1, MAG_BUS_2, MagnetometerId};
use crate::signals::RAW_MAG_CHANNELS;

const COLLECT_WINDOW: Duration = Duration::from_secs(30);

// deci-uT keeps the ~50 uT field well inside i16; results scale back to nT.
const NT_TO_DECI_UT: f32 = 1e-2;
const DECI_UT_TO_NT: f32 = 100.0;

const MIN_VALID_NT: f32 = 22_000.0;
const MAX_VALID_NT: f32 = 67_000.0;

#[embassy_executor::task]
pub async fn task(params: &'static Access) -> ! {
    let mut requests = RUN_MAG_CAL
        .subscribe()
        .expect("mag cal: RUN_MAG_CAL subscribe failed");

    loop {
        if !requests.changed().await {
            continue;
        }

        info!(
            "mag cal: starting collection ({} s tumble window)",
            COLLECT_WINDOW.as_secs()
        );

        let mut solver_0 = Solver::new();
        let mut solver_1 = Solver::new();
        collect_window(COLLECT_WINDOW, &mut solver_0, &mut solver_1).await;
        info!(
            "mag cal: collected {} / {} samples",
            solver_0.sample_count(),
            solver_1.sample_count()
        );

        let cal_0 = fit(&mut solver_0, MAG_BUS_1);
        let cal_1 = fit(&mut solver_1, MAG_BUS_2);

        store(params, &MAG_0_PARAM, MAG_BUS_1, cal_0).await;
        store(params, &MAG_1_PARAM, MAG_BUS_2, cal_1).await;

        let _ = params.set(&RUN_MAG_CAL, false).await;
    }
}

async fn collect_window(window: Duration, solver_0: &mut Solver, solver_1: &mut Solver) {
    let mut sub_0 = RAW_MAG_CHANNELS[MAG_BUS_1.index()]
        .subscriber()
        .expect("mag cal: raw mag 0 subscribe failed");
    let mut sub_1 = RAW_MAG_CHANNELS[MAG_BUS_2.index()]
        .subscriber()
        .expect("mag cal: raw mag 1 subscribe failed");

    let deadline = Instant::now() + window;
    while Instant::now() < deadline {
        let remaining = deadline - Instant::now();
        let next = select(sub_0.next_message_pure(), sub_1.next_message_pure());
        match with_timeout(remaining, next).await {
            Ok(Either::First(s)) => solver_0.push_sample(to_board_counts(s.x, s.y, s.z)),
            Ok(Either::Second(s)) => solver_1.push_sample(to_board_counts(s.x, s.y, s.z)),
            Err(_) => break,
        }
    }
}

// Negation is the sensor-to-board remap (see apply_calibration); fit in board frame.
fn to_board_counts(x: f32, y: f32, z: f32) -> [i16; 3] {
    [x, y, z].map(|v| (-v * NT_TO_DECI_UT) as i16)
}

fn fit(solver: &mut Solver, id: MagnetometerId) -> Option<MagCalWire> {
    let cal = match solver.solve() {
        Ok(cal) => cal,
        Err(_) => {
            warn!("{}: fit failed, too few samples", id);
            return None;
        }
    };

    let field_nt = cal.field_strength * DECI_UT_TO_NT;
    info!(
        "{}: fit {} B={=f32} nT err={=f32} %",
        id,
        Debug2Format(&cal.tier),
        field_nt,
        cal.fit_error_percent
    );

    if !(MIN_VALID_NT..=MAX_VALID_NT).contains(&field_nt) {
        warn!("{}: implausible field {=f32} nT, discarding", id, field_nt);
        return None;
    }

    let s = cal.soft_iron;
    Some(MagCalWire {
        hard_iron: cal.hard_iron.map(|h| h * DECI_UT_TO_NT),
        soft_iron: [
            s[0][0], s[0][1], s[0][2], s[1][0], s[1][1], s[1][2], s[2][0], s[2][1], s[2][2],
        ],
    })
}

async fn store(
    params: &Access,
    param: &Param<CriticalSectionRawMutex, MagCalWire>,
    id: MagnetometerId,
    cal: Option<MagCalWire>,
) {
    let Some(cal) = cal else { return };
    if params.set(param, cal).await.is_err() {
        warn!("{}: param write failed", id);
    }
}
