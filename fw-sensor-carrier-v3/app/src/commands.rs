//! Volatile command parameters. Setting one to `true` asks the
//! corresponding task to act and clear it back to `false`.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use firmware_params::{Param, make_key};

type Mutex = CriticalSectionRawMutex;

pub static RESET: Param<Mutex, bool, 1> = Param::new(make_key("v1/cmd/reset"));
pub static RUN_MAG_CAL: Param<Mutex, bool, 1> = Param::new(make_key("v1/cmd/run_mag_cal"));
pub static RUN_IMU_CAL: Param<Mutex, bool, 1> = Param::new(make_key("v1/cmd/run_imu_cal"));
