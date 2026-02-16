#![no_std]

pub mod sensor_def;

use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::usart::{
    ConfigError, Instance, InterruptHandler, RxDma, RxPin, TxDma, TxPin, Uart, UartRx, UartTx,
};
use embassy_stm32::{Peri, usart};
use embassy_time::{Duration, block_for, with_timeout};

use bitflags::bitflags;
use core::fmt;
use heapless::Vec as HVec;
use rmodbus::{ModbusProto, client::ModbusRequest, guess_response_frame_len};

/// Keller 23SX register addresses (subset + utility)
mod k23 {
    pub const REG_P1: u16 = 0x0002; // f32
    pub const REG_P2: u16 = 0x0004; // f32
    pub const REG_T: u16 = 0x0006; // f32
    pub const REG_TOB_1: u16 = 0x0008; // f32
    pub const REG_TOB_2: u16 = 0x000A; // f32

    pub const REG_P1_TOB1_BASE: u16 = 0x0100; // [f32; 2]
    pub const REG_P2_TOB2_BASE: u16 = 0x0104; // [f32; 2]

    pub const REG_UART_CONFIG: u16 = 0x0200; // u16 (RW)
    pub const REG_SN: u16 = 0x0202; // u32
    pub const REG_CFG_P: u16 = 0x0204; // u16 RO
    pub const REG_CFG_T: u16 = 0x0205; // u16 RO
    pub const REG_STATUS: u16 = 0x020C; // u16 RO
    pub const REG_DEV_ADDR: u16 = 0x020D; // u16 (1..=247)
    pub const REG_FW_CLASS: u16 = 0x020E; // u16 RO
    pub const REG_FW_DATE: u16 = 0x020F; // u16 RO
}

/// Error type
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KellerSensError {
    Proto,
    InvalidData,
    Timeout,
    AddrRange,
    ConfigError(ConfigError),
    UartSettings(UartSettingsError),
}

/// The Keller driver (bus + timeout; unit_id is passed per call).
/// Shared RTU bus: owns UART + DE line.
pub struct KellerSensRS485<'d> {
    tx: UartTx<'d, embassy_stm32::mode::Async>,
    rx: UartRx<'d, embassy_stm32::mode::Async>,
    de: Output<'d>,
    baud: u32,
    timeout: Duration,
}

impl<'d> KellerSensRS485<'d> {
    pub async fn new<UART: Instance>(
        peri: Peri<'d, UART>,
        rx: Peri<'d, impl RxPin<UART>>,
        tx: Peri<'d, impl TxPin<UART>>,
        tx_dma: Peri<'d, impl TxDma<UART>>,
        rx_dma: Peri<'d, impl RxDma<UART>>,
        irq: impl Binding<UART::Interrupt, InterruptHandler<UART>> + 'd,
        de_pin: Peri<'d, impl Pin>,
    ) -> Result<Self, ConfigError> {
        let mut cfg = usart::Config::default();
        cfg.baudrate = 115_200;

        let uart = Uart::new(peri, rx, tx, irq, tx_dma, rx_dma, cfg)?;
        let (tx, rx) = uart.split();

        let de = Output::new(de_pin, Level::Low, Speed::VeryHigh);

        Ok(Self {
            tx,
            rx,
            de,
            baud: 115_200,
            timeout: Duration::from_millis(10),
        })
    }

    fn rtu_gaps(&self) -> (Duration, Duration) {
        // t_char ≈ 10/baud
        let t_char_us = 10_000_000u32.div_ceil(self.baud);
        let t15 = Duration::from_micros((t_char_us as u64 * 15) / 10);
        let t35 = Duration::from_micros((t_char_us as u64 * 35) / 10);
        (t15, t35)
    }

    async fn transact(&mut self, req: &[u8], dst: &mut HVec<u8, 256>) -> Result<(), ()> {
        let (t15, t35) = self.rtu_gaps();

        // NOTE: This blocking call is intentional to guarantee correct timing.
        // Do not "optimize" it away — it will break things!
        // (Lesson learned the hard way: after wasting hours debugging and
        // eventually scrapping a VIF, 3 Oct 2025.)
        // — Louis & Domenic
        self.de.set_high();
        block_for(t15); // todo: possibly this could be an async timer instead

        self.tx.blocking_write(req).map_err(|_| ())?;
        self.tx.blocking_flush().map_err(|_| ())?;

        block_for(t35); // todo: this one could probably be ignored, but might interfere with the idle detection.
        self.de.set_low();

        let mut buf = [0u8; 256];
        let n = with_timeout(self.timeout, self.rx.read_until_idle(&mut buf))
            .await
            .map_err(|_| ())? // timeout wrapper
            .map_err(|_| ())?; // usart error

        dst.extend_from_slice(&buf[..n]).map_err(|_| ())?;
        if n >= 6
            && let Ok(exp) = guess_response_frame_len(dst, ModbusProto::Rtu)
            && dst.len() < exp as usize
        {
            return Err(()); // truncated; enlarge buf or timeout was too short
        }
        Ok(())
    }
}

impl<'d> KellerSensRS485<'d> {
    /// Read N holding registers from a given unit id.
    async fn read_holdings<const N: usize>(
        &mut self,
        unit_id: u8,
        reg: u16,
        count: u16,
    ) -> Result<HVec<u16, N>, KellerSensError> {
        let mut mreq = ModbusRequest::new(unit_id, ModbusProto::Rtu);

        let mut req = HVec::<u8, 256>::new();
        mreq.generate_get_holdings(reg, count, &mut req)
            .map_err(|_| KellerSensError::Proto)?;

        let mut resp = HVec::<u8, 256>::new();
        self.transact(req.as_slice(), &mut resp)
            .await
            .map_err(|_| KellerSensError::Timeout)?;

        let mut words = HVec::<u16, N>::new();
        mreq.parse_u16(resp.as_slice(), &mut words)
            .map_err(|_| KellerSensError::Proto)?;
        if words.len() != count as usize {
            return Err(KellerSensError::InvalidData);
        }
        Ok(words)
    }

    /// Write one register.
    async fn write_single(
        &mut self,
        unit_id: u8,
        reg: u16,
        value: u16,
    ) -> Result<(), KellerSensError> {
        let mut mreq = ModbusRequest::new(unit_id, ModbusProto::Rtu);
        let mut req = HVec::<u8, 256>::new();
        mreq.generate_set_holding(reg, value, &mut req)
            .map_err(|_| KellerSensError::Proto)?;

        let mut resp = HVec::<u8, 256>::new();
        self.transact(req.as_slice(), &mut resp)
            .await
            .map_err(|_| KellerSensError::Timeout)?;
        mreq.parse_ok(resp.as_slice())
            .map_err(|_| KellerSensError::Proto)
    }

    #[inline]
    fn u32_from_words(hi: u16, lo: u16) -> u32 {
        ((hi as u32) << 16) | (lo as u32)
    }
    #[inline]
    fn f32_from_words(hi: u16, lo: u16) -> f32 {
        f32::from_bits(Self::u32_from_words(hi, lo))
    }

    // ---- Basic measurements ----
    pub async fn read_p1(&mut self, unit_id: u8) -> Result<f32, KellerSensError> {
        let w = self.read_holdings::<2>(unit_id, k23::REG_P1, 2).await?;
        Ok(Self::f32_from_words(w[0], w[1]))
    }

    pub async fn read_p2(&mut self, unit_id: u8) -> Result<f32, KellerSensError> {
        let w = self.read_holdings::<2>(unit_id, k23::REG_P2, 2).await?;
        Ok(Self::f32_from_words(w[0], w[1]))
    }

    pub async fn read_t(&mut self, unit_id: u8) -> Result<f32, KellerSensError> {
        let w = self.read_holdings::<2>(unit_id, k23::REG_T, 2).await?;
        Ok(Self::f32_from_words(w[0], w[1]))
    }

    /// Ambient temperature at sensor 1 (TOB1) in °C.
    pub async fn read_tob1(&mut self, unit_id: u8) -> Result<f32, KellerSensError> {
        let w = self.read_holdings::<2>(unit_id, k23::REG_TOB_1, 2).await?;
        Ok(Self::f32_from_words(w[0], w[1]))
    }

    /// Ambient temperature at sensor 2 (TOB2) in °C.
    pub async fn read_tob2(&mut self, unit_id: u8) -> Result<f32, KellerSensError> {
        let w = self.read_holdings::<2>(unit_id, k23::REG_TOB_2, 2).await?;
        Ok(Self::f32_from_words(w[0], w[1]))
    }

    /// Bulk read (P1, TOB1).
    pub async fn read_p1_tob1(&mut self, unit_id: u8) -> Result<(f32, f32), KellerSensError> {
        let w = self
            .read_holdings::<4>(unit_id, k23::REG_P1_TOB1_BASE, 4)
            .await?;
        Ok((
            Self::f32_from_words(w[0], w[1]),
            Self::f32_from_words(w[2], w[3]),
        ))
    }

    /// Bulk read (P2, TOB2).
    pub async fn read_p2_tob2(&mut self, unit_id: u8) -> Result<(f32, f32), KellerSensError> {
        let w = self
            .read_holdings::<4>(unit_id, k23::REG_P2_TOB2_BASE, 4)
            .await?;
        Ok((
            Self::f32_from_words(w[0], w[1]),
            Self::f32_from_words(w[2], w[3]),
        ))
    }

    // ---- Identity / config ----
    pub async fn read_serial(&mut self, unit_id: u8) -> Result<u32, KellerSensError> {
        let w = self.read_holdings::<2>(unit_id, k23::REG_SN, 2).await?;
        Ok(Self::u32_from_words(w[0], w[1]))
    }

    pub async fn read_dev_addr(&mut self, unit_id: u8) -> Result<u8, KellerSensError> {
        let w = self
            .read_holdings::<1>(unit_id, k23::REG_DEV_ADDR, 1)
            .await?;
        let v = w[0];
        if (1..=247).contains(&v) {
            Ok(v as u8)
        } else {
            Err(KellerSensError::InvalidData)
        }
    }

    pub async fn write_dev_addr(&mut self, unit_id: u8, addr: u16) -> Result<(), KellerSensError> {
        if !(1..=247).contains(&addr) {
            return Err(KellerSensError::AddrRange);
        }
        self.write_single(unit_id, k23::REG_DEV_ADDR, addr).await
    }

    /// Read UART settings (baud/parity/stop) from device.
    pub async fn read_uart_settings(
        &mut self,
        unit_id: u8,
    ) -> Result<UartSettings, KellerSensError> {
        let w = self
            .read_holdings::<1>(unit_id, k23::REG_UART_CONFIG, 1)
            .await?;
        UartSettings::try_from(w[0]).map_err(KellerSensError::UartSettings)
    }

    /// Write UART settings to device.
    pub async fn write_uart_settings(
        &mut self,
        unit_id: u8,
        settings: UartSettings,
    ) -> Result<(), KellerSensError> {
        self.write_single(unit_id, k23::REG_UART_CONFIG, settings.to_bits())
            .await
    }

    /// Read pressure channel presence flags.
    pub async fn read_pressure_channel_flags(
        &mut self,
        unit_id: u8,
    ) -> Result<ActivePressure, KellerSensError> {
        let w = self.read_holdings::<1>(unit_id, k23::REG_CFG_P, 1).await?;
        Ok(ActivePressure::from_bits_truncate(w[0]))
    }

    /// Read temperature channel presence flags.
    pub async fn read_temperature_channel_flags(
        &mut self,
        unit_id: u8,
    ) -> Result<ActiveTemp, KellerSensError> {
        let w = self.read_holdings::<1>(unit_id, k23::REG_CFG_T, 1).await?;
        Ok(ActiveTemp::from_bits_truncate(w[0]))
    }

    /// Read device status/error flags.
    pub async fn read_status(&mut self, unit_id: u8) -> Result<Status, KellerSensError> {
        let w = self.read_holdings::<1>(unit_id, k23::REG_STATUS, 1).await?;
        Ok(Status::from_bits_truncate(w[0]))
    }

    /// Read firmware (class.group).
    pub async fn read_firmware_class_group(
        &mut self,
        unit_id: u8,
    ) -> Result<FirmwareClassGroup, KellerSensError> {
        let w = self
            .read_holdings::<1>(unit_id, k23::REG_FW_CLASS, 1)
            .await?;
        Ok(FirmwareClassGroup {
            class: (w[0] >> 8) as u8,
            group: (w[0] & 0xFF) as u8,
        })
    }

    /// Read firmware build date (20YY.WW).
    pub async fn read_firmware_date(
        &mut self,
        unit_id: u8,
    ) -> Result<FirmwareDate, KellerSensError> {
        let w = self
            .read_holdings::<1>(unit_id, k23::REG_FW_DATE, 1)
            .await?;
        Ok(FirmwareDate {
            year: (w[0] >> 8) as u8,
            week: (w[0] & 0xFF) as u8,
        })
    }
}

// ---------- UART settings + flags (no_std compatible) ----------

/// Error for invalid raw UART configuration values.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartSettingsError {
    /// Raw value has unknown bits outside the defined UART flags.
    UnknownBitsSet(u16),
    /// Reserved bits (1, 2, 3) are set in the raw value.
    ReservedBaudBitsSet(u16),
}

/// Baud rate configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BaudRate {
    Baud9600,
    Baud115200,
}

/// UART parity configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    None,
    Odd,
    Even,
}

/// UART stop bits configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    One,
    Two,
}

bitflags! {
    /// Each flag corresponds directly to a bit as defined in the datasheet.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct UartRegisterFlags: u16 {
        /// Baud rate setting for 115200 baud. If not set, baud rate is 9600.
        const BAUD_115200 = 0b0000_0000_0000_0001;
        // The remaining baud rate bits are reserved and must be zero.
        const BAUD_RESERVED = 0b0000_0000_0000_1110;

        /// Enables the use of a parity bit.
        const PARITY_ENABLE = 0b0000_0000_0001_0000;
        /// Selects the parity mode (0=Odd, 1=Even) when parity is enabled.
        const PARITY_MODE_EVEN = 0b0000_0000_0010_0000;

        /// Selects two stop bits instead of one.
        const STOP_BIT_TWO = 0b0000_0000_0100_0000;
    }
}

/// UART configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UartSettings {
    /// Baud rate setting
    pub baud_rate: BaudRate,
    /// Parity bit configuration
    pub parity: Parity,
    /// Number of stop bits
    pub stop_bits: StopBits,
}

impl UartSettings {
    pub const fn to_bits(self) -> u16 {
        let mut bits: u16 = 0;

        if matches!(self.baud_rate, BaudRate::Baud115200) {
            bits |= UartRegisterFlags::BAUD_115200.bits();
        }
        match self.parity {
            Parity::None => {}
            Parity::Odd => bits |= UartRegisterFlags::PARITY_ENABLE.bits(),
            Parity::Even => {
                bits |= UartRegisterFlags::PARITY_ENABLE.bits()
                    | UartRegisterFlags::PARITY_MODE_EVEN.bits()
            }
        }
        if matches!(self.stop_bits, StopBits::Two) {
            bits |= UartRegisterFlags::STOP_BIT_TWO.bits();
        }
        bits
    }
}

impl TryFrom<u16> for UartSettings {
    type Error = UartSettingsError;

    fn try_from(bits: u16) -> Result<Self, Self::Error> {
        let Some(flags) = UartRegisterFlags::from_bits(bits) else {
            return Err(UartSettingsError::UnknownBitsSet(bits));
        };
        if flags.intersects(UartRegisterFlags::BAUD_RESERVED) {
            return Err(UartSettingsError::ReservedBaudBitsSet(bits));
        }

        let baud_rate = if flags.contains(UartRegisterFlags::BAUD_115200) {
            BaudRate::Baud115200
        } else {
            BaudRate::Baud9600
        };
        let parity = match (
            flags.contains(UartRegisterFlags::PARITY_ENABLE),
            flags.contains(UartRegisterFlags::PARITY_MODE_EVEN),
        ) {
            (false, _) => Parity::None,
            (true, false) => Parity::Odd,
            (true, true) => Parity::Even,
        };
        let stop_bits = if flags.contains(UartRegisterFlags::STOP_BIT_TWO) {
            StopBits::Two
        } else {
            StopBits::One
        };

        Ok(UartSettings {
            baud_rate,
            parity,
            stop_bits,
        })
    }
}

// ---------- Device capabilities / status flags (no_std compatible) ----------

bitflags! {
    /// Pressure channels available on the device
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ActivePressure: u16 {
        /// Primary pressure sensor (P1)
        const P1 = 1 << 1;
        /// Secondary pressure sensor (P2)
        const P2 = 1 << 2;
    }
}

bitflags! {
    /// Temperature channels available on the device
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ActiveTemp: u16 {
        /// Internal temperature sensor (T)
        const T = 1 << 3;
        /// External temperature sensor 1 (TOB1)
        const TOB1 = 1 << 4;
        /// External temperature sensor 2 (TOB2)
        const TOB2 = 1 << 5;
    }
}

bitflags! {
    /// Device status and error flags, based on the STAT byte definition.
    ///
    /// The STAT byte is an 8-bit value, but is read as part of a 16-bit Modbus register.
    /// The upper 8 bits are not defined.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Status: u16 {
        /// A set CH0 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const CH0_ERROR = 1 << 0;

        /// A set P1 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const P1_ERROR = 1 << 1;

        /// A set P2 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const P2_ERROR = 1 << 2;

        /// A set T bit indicates that a measuring or computation error has occurred in the channel concerned.
        const T_ERROR = 1 << 3;

        /// A set TOB1 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const TOB1_ERROR = 1 << 4;

        /// A set TOB2 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const TOB2_ERROR = 1 << 5;

        /// Named `ERR2` in the datasheet.
        /// A set ERR2 bit denotes that a computation error has occurred in the calculation process for the analogue output.
        /// This occurs if the analogue Signal is in saturation (depends on the scaling).
        const ANALOGUE_OUT_ERROR = 1 << 6;

        /// Named `/STD` in the datasheet.
        /// A set /STD bit indicate whether the transmitter is in Power-up mode, otherwise it is in Standard mode.
        /// For version 5.21-XX.XX, /STD is used to indicate an error during measuring the conductivity.
        const POWER_UP_MODE = 1 << 7;
    }
}

impl Status {
    /// Check if the device has any channel or computation errors.
    pub fn has_errors(&self) -> bool {
        self.intersects(
            Self::CH0_ERROR
                | Self::P1_ERROR
                | Self::P2_ERROR
                | Self::T_ERROR
                | Self::TOB1_ERROR
                | Self::TOB2_ERROR
                | Self::ANALOGUE_OUT_ERROR,
        )
    }

    /// Check if the device is in the power-up state (not yet initialized).
    pub fn is_in_power_up_mode(&self) -> bool {
        self.contains(Self::POWER_UP_MODE)
    }
}

// ---------- Firmware info (no_std Display) ----------

/// Firmware version information (class.group format)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FirmwareClassGroup {
    /// Firmware class (major version)
    pub class: u8,
    /// Firmware group (minor version)
    pub group: u8,
}

impl fmt::Display for FirmwareClassGroup {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.class, self.group)
    }
}

/// Firmware build date information
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FirmwareDate {
    /// Year (2-digit format, e.g. 20 for 2020)
    pub year: u8,
    /// Week number (1-52)
    pub week: u8,
}

impl fmt::Display for FirmwareDate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "20{}.{:02}", self.year, self.week)
    }
}
