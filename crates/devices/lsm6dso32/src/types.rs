use crate::device::{AccelerometerFullScale, GyroscopeFullScale};

/// Raw accelerometer readings from the LSM6DSO32.
///
/// Each field (x, y, z) is a signed 16-bit integer (LSB).
/// These values must be converted to g (m/s²) using `Acceleration::from_raw`.
#[derive(Debug, Clone, Copy)]
pub struct AccelerationRaw {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Acceleration in g (1g ≈ 9.81 m/s²).
///
/// Each field (x, y, z) is a floating-point value representing the
/// measured acceleration after conversion from raw LSB.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Acceleration {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Acceleration {
    /// Converts raw accelerometer data (LSB) into g.
    pub fn from_raw(raw: AccelerationRaw, full_scale: AccelerometerFullScale) -> Self {
        let scale = match full_scale {
            AccelerometerFullScale::G4 => 8192.0,
            AccelerometerFullScale::G8 => 4096.0,
            AccelerometerFullScale::G16 => 2048.0,
            AccelerometerFullScale::G32 => 1024.0,
        };
        let x = raw.x as f32 / scale;
        let y = raw.y as f32 / scale;
        let z = raw.z as f32 / scale;
        Self { x, y, z }
    }
}

/// Raw gyroscope readings from the LSM6DSO32.
///
/// Each field (x, y, z) is a signed 16-bit integer (LSB).
/// These values must be converted to dps using `AngularRate::from_raw`.
#[derive(Debug, Clone, Copy)]
pub struct AngularRateRaw {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Angular rate in degrees per second (dps).
///
/// Each field (x, y, z) is a floating-point value representing the
/// measured rotation rate after conversion from raw LSB.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AngularRate {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl AngularRate {
    /// Converts raw gyroscope data (LSB) into degrees per second (dps).
    ///
    /// Direct multiplication factors (datasheet):
    /// - ±250 dps  → 8.75 mdps/LSB  → 0.00875 dps/LSB
    /// - ±500 dps  → 17.50 mdps/LSB → 0.01750 dps/LSB
    /// - ±1000 dps → 35.00 mdps/LSB → 0.03500 dps/LSB
    /// - ±2000 dps → 70.00 mdps/LSB → 0.07000 dps/LSB
    pub fn from_raw(raw: AngularRateRaw, full_scale: GyroscopeFullScale) -> Self {
        let dps_per_lsb = match full_scale {
            GyroscopeFullScale::Dps250 => 0.00875,
            GyroscopeFullScale::Dps500 => 0.01750,
            GyroscopeFullScale::Dps1000 => 0.03500,
            GyroscopeFullScale::Dps2000 => 0.07000,
        };

        let x = raw.x as f32 * dps_per_lsb;
        let y = raw.y as f32 * dps_per_lsb;
        let z = raw.z as f32 * dps_per_lsb;

        Self { x, y, z }
    }
}

/// Raw temperature reading from the LSM6DSO32.
///
/// `value` is a signed 16-bit integer representing an offset from 25°C.
/// Should be converted to °C using `Temperature::from_raw`.
#[derive(Debug, Clone, Copy)]
pub struct TemperatureRaw {
    pub value: i16,
}

/// Temperature in degrees Celsius (°C).
///
/// `value` is a floating-point representation of the sensor reading.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Temperature {
    pub value: f32,
}

impl Temperature {
    /// Converts raw temperature data (LSB) into °C.
    pub fn from_raw(raw: TemperatureRaw) -> Self {
        let value = 25.0 + (raw.value as f32 / 256.0);
        Self { value }
    }
}

/// Raw FIFO data from the LSM6DSO32.
#[repr(C)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct FifoDataOut {
    bits: [u8; 7],
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for FifoDataOut {
    fn format(&self, fmt: defmt::Formatter) {
        let tag = self.tag_sensor();
        defmt::write!(
            fmt,
            "FifoDataOut {{ tag: {:?}, x: {}, y: {}, z: {} }}",
            tag,
            self.x(),
            self.y(),
            self.z()
        );
    }
}

impl FifoDataOut {
    pub fn new_with_zero() -> Self {
        Self { bits: [0u8; 7] }
    }

    #[inline(always)]
    pub fn tag_sensor(&self) -> TagSensor {
        let raw = self.bits[0] >> 3;
        TagSensor::from_raw(raw)
    }

    #[inline(always)]
    pub fn tag_cnt(&self) -> u8 {
        (self.bits[0] >> 1) & 0b11
    }

    #[inline(always)]
    pub fn tag_parity(&self) -> bool {
        (self.bits[0] & 0b1) != 0
    }

    #[inline(always)]
    pub fn x(&self) -> i16 {
        i16::from_le_bytes([self.bits[1], self.bits[2]])
    }

    #[inline(always)]
    pub fn y(&self) -> i16 {
        i16::from_le_bytes([self.bits[3], self.bits[4]])
    }

    #[inline(always)]
    pub fn z(&self) -> i16 {
        i16::from_le_bytes([self.bits[5], self.bits[6]])
    }

    #[inline(always)]
    pub fn timestamp(&self) -> u32 {
        u32::from_le_bytes([self.bits[1], self.bits[2], self.bits[3], self.bits[4]])
    }

    #[inline(always)]
    pub fn temperature(&self) -> i16 {
        i16::from_le_bytes([self.bits[1], self.bits[2]])
    }

    #[inline(always)]
    pub fn data(&self) -> &[u8] {
        &self.bits[1..7]
    }
}

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum TagSensor {
    GyroscopeNC = 0x01,
    AccelerometerNC = 0x02,
    Temperature = 0x03,
    Timestamp = 0x04,
    CFGChange = 0x05,
    AccelerometerNC_T_2 = 0x06,
    AccelerometerNC_T_1 = 0x07,
    Accelerometer2xC = 0x08,
    Accelerometer3xC = 0x09,
    GyroscopeNC_T_2 = 0x0A,
    GyroscopeNC_T_1 = 0x0B,
    Gyroscope2xC = 0x0C,
    Gyroscope3xC = 0x0D,
    SensorHubSlave0 = 0x0E,
    SensorHubSlave1 = 0x0F,
    SensorHubSlave2 = 0x10,
    SensorHubSlave3 = 0x11,
    StepCounter = 0x12,
    SensorHubNack = 0x19,
    /// Represents any value not matching the above definitions.
    Unknown(u8),
}

impl TagSensor {
    /// Converts a raw 5-bit value into a [`TagSensor`] enum.
    #[inline(always)]
    pub fn from_raw(raw: u8) -> Self {
        match raw {
            0x01 => TagSensor::GyroscopeNC,
            0x02 => TagSensor::AccelerometerNC,
            0x03 => TagSensor::Temperature,
            0x04 => TagSensor::Timestamp,
            0x05 => TagSensor::CFGChange,
            0x06 => TagSensor::AccelerometerNC_T_2,
            0x07 => TagSensor::AccelerometerNC_T_1,
            0x08 => TagSensor::Accelerometer2xC,
            0x09 => TagSensor::Accelerometer3xC,
            0x0A => TagSensor::GyroscopeNC_T_2,
            0x0B => TagSensor::GyroscopeNC_T_1,
            0x0C => TagSensor::Gyroscope2xC,
            0x0D => TagSensor::Gyroscope3xC,
            0x0E => TagSensor::SensorHubSlave0,
            0x0F => TagSensor::SensorHubSlave1,
            0x10 => TagSensor::SensorHubSlave2,
            0x11 => TagSensor::SensorHubSlave3,
            0x12 => TagSensor::StepCounter,
            0x19 => TagSensor::SensorHubNack,
            other => TagSensor::Unknown(other),
        }
    }
}
