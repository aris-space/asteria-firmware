use crate::modbus_server::{DEVICE_STATUS, DeviceStatus};
use core::sync::atomic::Ordering;
use embassy_stm32::gpio::{Level, Output};
use embassy_time::{Duration, Ticker};

type LedStates = (Level, Level); // (Red, Green)
type LedPattern = &'static [LedStates; 20];

const ON: Level = Level::High;
const OFF: Level = Level::Low;

const OFF_PATTERN: &[LedStates; 20] = &[(OFF, OFF); 20];

const BLINK_FAST_GREEN_SOLID_RED: &[LedStates; 20] = &[
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
];

// Red is OFF, Green blinks fast.
const BLINK_FAST_GREEN: &[LedStates; 20] = &[
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
];

const BLINK_SLOW_GREEN: &[LedStates; 20] = &[
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, ON),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
];

const BLINK_SLOW_RED: &[LedStates; 20] = &[
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
    (OFF, OFF),
];

const BLINK_SLOW_GREEN_SOLID_RED: &[LedStates; 20] = &[
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, ON),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
    (ON, OFF),
];

fn select_pattern(status: DeviceStatus) -> LedPattern {
    if status.contains(DeviceStatus::MOTOR_INIT_FAILED) {
        return BLINK_SLOW_RED;
    }
    if status.contains(DeviceStatus::INITIALIZING)
        && status.intersects(DeviceStatus::MOTOR_COMM_ERROR | DeviceStatus::MOTOR_DEVICE_FAULT)
    {
        return BLINK_FAST_GREEN_SOLID_RED;
    }
    if status.contains(DeviceStatus::READY)
        && status.intersects(DeviceStatus::MOTOR_COMM_ERROR | DeviceStatus::MOTOR_DEVICE_FAULT)
    {
        return BLINK_SLOW_GREEN_SOLID_RED;
    }
    if status.contains(DeviceStatus::INITIALIZING) {
        return BLINK_FAST_GREEN;
    }
    if status.contains(DeviceStatus::READY) {
        return BLINK_SLOW_GREEN;
    }
    OFF_PATTERN
}

#[embassy_executor::task]
pub async fn status_blinky(mut green: Output<'static>, mut red: Output<'static>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let mut cycle_tick = 0_usize;

    loop {
        let status_bits = DEVICE_STATUS.load(Ordering::Relaxed);
        let status = DeviceStatus::from_bits_truncate(status_bits);
        let pattern = select_pattern(status);

        let (red_state, green_state) = pattern[cycle_tick];
        red.set_level(red_state);
        green.set_level(green_state);

        cycle_tick = (cycle_tick + 1) % pattern.len();

        ticker.next().await;
    }
}
