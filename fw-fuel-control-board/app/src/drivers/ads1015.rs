use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

const DEFAULT_ADDRESS: u8 = 0x48;

const REG_CONVERSION: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;

const CONFIG_OS_SINGLE: u16 = 0x8000;
const CONFIG_PGA_4V096: u16 = 0x0200;
const CONFIG_MODE_SINGLE_SHOT: u16 = 0x0100;
const CONFIG_DATA_RATE_1600_SPS: u16 = 0x0080;
const CONFIG_COMP_DISABLE: u16 = 0x0003;

const AIN_SINGLE_ENDED_BASE: u16 = 0x4000;
const AIN_SINGLE_ENDED_STEP: u16 = 0x1000;

const FULL_SCALE_MILLIVOLTS: f32 = 4096.0;
const ADS1015_COUNTS: f32 = 2048.0;

pub struct Ads1015<I> {
    i2c: I,
    address: u8,
}

impl<I> Ads1015<I> {
    pub const fn new(i2c: I) -> Self {
        Self {
            i2c,
            address: DEFAULT_ADDRESS,
        }
    }
}

impl<I, E> Ads1015<I>
where
    I: I2c<Error = E>,
{
    pub async fn read_single_ended(&mut self, channel: SingleEndedChannel) -> Result<AdcSample, E> {
        let config = CONFIG_OS_SINGLE
            | channel.mux_bits()
            | CONFIG_PGA_4V096
            | CONFIG_MODE_SINGLE_SHOT
            | CONFIG_DATA_RATE_1600_SPS
            | CONFIG_COMP_DISABLE;

        self.write_register(REG_CONFIG, config).await?;
        Timer::after(Duration::from_millis(1)).await;

        let raw = self.read_register(REG_CONVERSION).await?;
        let counts = raw >> 4;

        Ok(AdcSample {
            raw_counts: counts,
            millivolts: counts as f32 * FULL_SCALE_MILLIVOLTS / ADS1015_COUNTS,
        })
    }

    async fn write_register(&mut self, register: u8, value: u16) -> Result<(), E> {
        self.i2c
            .write(
                self.address,
                &[register, (value >> 8) as u8, (value & 0xff) as u8],
            )
            .await
    }

    async fn read_register(&mut self, register: u8) -> Result<u16, E> {
        let mut buf = [0; 2];
        self.i2c
            .write_read(self.address, &[register], &mut buf)
            .await?;
        Ok(u16::from_be_bytes(buf))
    }
}

#[derive(Clone, Copy)]
pub enum SingleEndedChannel {
    A0,
    A1,
    A2,
    #[allow(dead_code)]
    A3,
}

impl SingleEndedChannel {
    const fn mux_bits(self) -> u16 {
        AIN_SINGLE_ENDED_BASE
            + match self {
                Self::A0 => 0,
                Self::A1 => AIN_SINGLE_ENDED_STEP,
                Self::A2 => 2 * AIN_SINGLE_ENDED_STEP,
                Self::A3 => 3 * AIN_SINGLE_ENDED_STEP,
            }
    }
}

#[derive(Clone, Copy)]
pub struct AdcSample {
    pub raw_counts: u16,
    pub millivolts: f32,
}
