/// Number of independent IMU estimator chains.
pub const IMU_COUNT: usize = 2;

/// Identifies one IMU and its independent estimator chain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuId(u8);

impl ImuId {
    /// Every supported IMU ID in index order.
    pub const ALL: [Self; IMU_COUNT] = [IMU_0, IMU_1];

    /// Converts a sensor-family index into an ID.
    #[must_use]
    pub const fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(IMU_0),
            1 => Some(IMU_1),
            _ => None,
        }
    }

    /// Returns the array index used for this sensor family.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// First IMU on the sensor carrier.
pub const IMU_0: ImuId = ImuId(0);
/// Second IMU on the sensor carrier.
pub const IMU_1: ImuId = ImuId(1);

/// Number of barometers represented in the filter state.
pub const BAROMETER_COUNT: usize = 2;

/// Identifies one barometer and its bias state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BarometerId(u8);

impl BarometerId {
    /// Every supported barometer ID in index order.
    pub const ALL: [Self; BAROMETER_COUNT] = [BARO_BUS_1, BARO_BUS_2];

    /// Converts a sensor-family index into an ID.
    #[must_use]
    pub const fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(BARO_BUS_1),
            1 => Some(BARO_BUS_2),
            _ => None,
        }
    }

    /// Returns the array index used for this sensor family.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Barometer on sensor-carrier bus one.
pub const BARO_BUS_1: BarometerId = BarometerId(0);
/// Barometer on sensor-carrier bus two.
pub const BARO_BUS_2: BarometerId = BarometerId(1);
