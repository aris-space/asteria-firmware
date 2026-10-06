/// Shared interface of the per-family sensor ids, for code that handles
/// every sensor family the same way.
pub trait SensorId: Copy + defmt::Format + 'static {
    fn index(self) -> usize;
    fn name(self) -> &'static str;
    fn all() -> &'static [Self];

    fn from_name(name: &str) -> Option<Self> {
        Self::all().iter().copied().find(|id| id.name() == name)
    }
}

macro_rules! define_sensor_family {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident;
        count: $count_name:ident;
        ids: [$($id_name:ident = $index:literal),+ $(,)?];
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, PartialEq, Eq)]
        $vis struct $name(u8);

        impl $name {
            pub const ALL: [$name; $count_name] = [$($name($index)),+];

            pub const fn index(self) -> usize {
                self.0 as usize
            }

            /// The id's Rust identifier, e.g. `"IMU_0"`. Used for logs, console
            /// display, and flash keys, so there is a single source.
            pub const fn name(self) -> &'static str {
                match self.0 {
                    $($index => stringify!($id_name),)+
                    _ => "<unknown>",
                }
            }

        }

        impl SensorId for $name {
            fn index(self) -> usize {
                $name::index(self)
            }

            fn name(self) -> &'static str {
                $name::name(self)
            }

            fn all() -> &'static [Self] {
                &Self::ALL
            }
        }

        impl core::fmt::Debug for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str(self.name())
            }
        }

        impl defmt::Format for $name {
            fn format(&self, fmt: defmt::Formatter) {
                defmt::write!(fmt, "{=str}", self.name())
            }
        }

        $vis const $count_name: usize = [$(stringify!($id_name)),+].len();
        $( $vis const $id_name: $name = $name($index); )+
    };
}

define_sensor_family! {
    pub struct ImuId;
    count: IMU_COUNT;
    ids: [IMU_0 = 0, IMU_1 = 1];
}

define_sensor_family! {
    pub struct BaroId;
    count: BARO_COUNT;
    ids: [BARO_BUS_1 = 0, BARO_BUS_2 = 1];
}

define_sensor_family! {
    pub struct MagId;
    count: MAG_COUNT;
    ids: [MAG_BUS_1 = 0, MAG_BUS_2 = 1];
}

define_sensor_family! {
    pub struct GnssId;
    count: GNSS_COUNT;
    ids: [GNSS_1 = 0, GNSS_2 = 1];
}

define_sensor_family! {
    pub struct DhtId;
    count: DHT_COUNT;
    ids: [DHT_BUS_1 = 0, DHT_BUS_2 = 1];
}

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
status_array!(BARO_STATUS, BARO_COUNT);
status_array!(MAG_STATUS, MAG_COUNT);
status_array!(GNSS_STATUS, GNSS_COUNT);
status_array!(DHT_STATUS, DHT_COUNT);
