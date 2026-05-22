macro_rules! define_sensor_family {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident;
        count: $count_name:ident = $count:expr;
        ids: [$($id_name:ident = $index:expr),+ $(,)?];
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
        $vis struct $name(usize);

        impl $name {
            pub const fn index(self) -> usize {
                self.0
            }
        }

        $vis const $count_name: usize = $count;
        $( $vis const $id_name: $name = $name($index); )+
    };
}

define_sensor_family! {
    pub struct ImuId;
    count: IMU_COUNT = 2;
    ids: [IMU_0 = 0, IMU_1 = 1];
}

define_sensor_family! {
    pub struct BarometerId;
    count: BAROMETER_COUNT = 2;
    ids: [BAROMETER_0 = 0, BAROMETER_1 = 1];
}

define_sensor_family! {
    pub struct MagnetometerId;
    count: MAGNETOMETER_COUNT = 2;
    ids: [MAGNETOMETER_0 = 0, MAGNETOMETER_1 = 1];
}

define_sensor_family! {
    pub struct GnssId;
    count: GNSS_COUNT = 2;
    ids: [GNSS_0 = 0, GNSS_1 = 1];
}

define_sensor_family! {
    pub struct DhtId;
    count: DHT_COUNT = 2;
    ids: [DHT_0 = 0, DHT_1 = 1];
}

// --- Health tracking (CAN status frame) -------------------------------------

#[atomic_enum::atomic_enum]
pub enum SensorStatus {
    Inactive,
    Active,
}

impl From<SensorStatus> for hermes_can::messages::board_status::SensorStatus {
    fn from(value: SensorStatus) -> Self {
        match value {
            SensorStatus::Inactive => hermes_can::messages::board_status::SensorStatus::Offline,
            SensorStatus::Active => hermes_can::messages::board_status::SensorStatus::Online,
        }
    }
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
