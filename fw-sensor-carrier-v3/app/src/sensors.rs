macro_rules! define_sensor_family {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident;
        count: $count_name:ident;
        ids: [$($id_name:ident = $index:literal => $label:literal),+ $(,)?];
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, PartialEq, Eq)]
        $vis struct $name(u8);

        impl $name {
            pub const fn index(self) -> usize {
                self.0 as usize
            }

            pub const fn name(self) -> &'static str {
                match self.0 {
                    $($index => $label,)+
                    _ => "<unknown>",
                }
            }
        }

        impl core::fmt::Debug for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str(self.name())
            }
        }

        impl defmt::Format for $name {
            fn format(&self, fmt: defmt::Formatter) {
                let label = match self.0 {
                    $($index => defmt::intern!($label),)+
                    _ => defmt::intern!("<unknown>"),
                };
                defmt::write!(fmt, "{=istr}", label)
            }
        }

        $vis const $count_name: usize = [$(stringify!($id_name)),+].len();
        $( $vis const $id_name: $name = $name($index); )+
    };
}

define_sensor_family! {
    pub struct ImuId;
    count: IMU_COUNT;
    ids: [IMU_0 = 0 => "IMU 0",IMU_1 = 1 => "IMU 1",];
}

define_sensor_family! {
    pub struct BarometerId;
    count: BAROMETER_COUNT;
    ids: [BARO_BUS_1 = 0 => "Baro Bus 1", BARO_BUS_2 = 1 => "Baro Bus 2"];
}

define_sensor_family! {
    pub struct MagnetometerId;
    count: MAGNETOMETER_COUNT;
    ids: [MAG_BUS_1 = 0 => "Mag Bus 1", MAG_BUS_2 = 1 => "Mag Bus 2"];
}

define_sensor_family! {
    pub struct GnssId;
    count: GNSS_COUNT;
    ids: [GNSS_0 = 0 => "GNSS 0", GNSS_1 = 1 => "GNSS 1"];
}

define_sensor_family! {
    pub struct DhtId;
    count: DHT_COUNT;
    ids: [DHT_BUS_1 = 0 => "DHT Bus 1", DHT_BUS_2 = 1 => "DHT Bus 2"];
}

// --- Health tracking (CAN status frame) -------------------------------------

#[atomic_enum::atomic_enum]
pub enum SensorStatus {
    Inactive,
    Active,
}

macro_rules! status_array {
    ($name:ident, $count:expr) => {
        pub static $name: [AtomicSensorStatus; $count] =
            [const { AtomicSensorStatus::new(SensorStatus::Inactive) }; $count];
    };
}

status_array!(IMU_STATUS, IMU_COUNT);
status_array!(BAROMETER_STATUS, BAROMETER_COUNT);
status_array!(MAGNETOMETER_STATUS, MAGNETOMETER_COUNT);
status_array!(GNSS_STATUS, GNSS_COUNT);
status_array!(DHT_STATUS, DHT_COUNT);
