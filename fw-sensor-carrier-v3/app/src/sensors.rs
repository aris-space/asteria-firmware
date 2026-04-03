macro_rules! define_sensor_family {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident;
        count: $count_name:ident = $count:expr;
        all: $all_name:ident = [$($id_name:ident = $index:expr),+ $(,)?];
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
        $vis struct $name(usize);

        impl $name {
            pub const COUNT: usize = $count;

            pub const fn index(self) -> usize {
                self.0
            }
        }

        $vis const $count_name: usize = $count;
        $( $vis const $id_name: $name = $name($index); )+
        $vis const $all_name: [$name; $count] = [$($id_name),+];
    };
}

define_sensor_family! {
    pub struct ImuId;
    count: IMU_COUNT = 2;
    all: IMU_IDS = [IMU_0 = 0, IMU_1 = 1];
}

define_sensor_family! {
    pub struct BarometerId;
    count: BAROMETER_COUNT = 2;
    all: BAROMETER_IDS = [BAROMETER_0 = 0, BAROMETER_1 = 1];
}

define_sensor_family! {
    pub struct GnssId;
    count: GNSS_COUNT = 2;
    all: GNSS_IDS = [GNSS_0 = 0, GNSS_1 = 1];
}
