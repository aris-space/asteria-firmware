use crate::types::{
    Acceleration, AccelerationRaw, AngularRate, AngularRateRaw, Temperature, TemperatureRaw,
};
use core::cmp::min;

use crate::device::{
    AccelerometerFullScale, AccelerometerOdr, GyroscopeFullScale, GyroscopeOdr, Lsm6dso32device,
};
use crate::field_sets::StatusReg;
use crate::{
    AccelBatchDataRate, DecTsBatch, FifoDataOut, FifoMode, GyroBatchDataRate, TempBatchDataRate,
};
use core::marker::PhantomData;
use device_driver::AsyncRegisterInterface;
use embedded_hal_async::delay::DelayNs;

#[allow(dead_code)]
pub struct Initialised;
#[allow(dead_code)]
pub struct Uninitialised;
const WHOAMI: u8 = 0x6C;

pub struct Lsm6dso32<I, S> {
    inner: Lsm6dso32device<I>,
    /// valid as soon as the device is initialised
    accel_full_scale: AccelerometerFullScale,
    /// valid as soon as the device is initialised
    gyro_full_scale: GyroscopeFullScale,
    _state: PhantomData<S>,
}

pub struct InitError<I, E> {
    pub kind: InitErrorKind<E>,
    pub sensor: Lsm6dso32<I, Uninitialised>,
}

impl<I> core::fmt::Debug for InitError<I, <I as AsyncRegisterInterface>::Error>
where
    I: AsyncRegisterInterface<AddressType = u8>,
    <I as AsyncRegisterInterface>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self.kind)
    }
}

// Optional defmt support
#[cfg(feature = "defmt-03")]
impl<I> defmt::Format for InitError<I, <I as AsyncRegisterInterface>::Error>
where
    I: AsyncRegisterInterface<AddressType = u8>,
    <I as AsyncRegisterInterface>::Error: defmt::Format,
{
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:?}", self.kind);
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum InitErrorKind<E> {
    /// Bus error occurred during initialisation.
    BusError(E),
    /// WHOAMI register mismatch. If this error occurs, the device is either
    /// not an LSM6DSO32 or may not be functioning correctly.
    WhoamiMismatch,
}

impl<I, S> Lsm6dso32<I, S> {
    /// Create a new LSM6DSO32 driver from the given interface.
    pub fn new(interface: I) -> Lsm6dso32<I, Uninitialised> {
        Lsm6dso32 {
            inner: Lsm6dso32device::new(interface),
            accel_full_scale: AccelerometerFullScale::G4,
            gyro_full_scale: GyroscopeFullScale::Dps250,
            _state: PhantomData,
        }
    }

    /// Returns an immutable handle to the inner device.
    pub fn inner(&self) -> &Lsm6dso32device<I> {
        &self.inner
    }

    /// Returns a mutable handle to the inner device.
    pub fn inner_mut(&mut self) -> &mut Lsm6dso32device<I> {
        &mut self.inner
    }

    /// Consumes the device and returns the inner interface.
    pub fn destroy(self) -> I {
        self.inner.interface
    }

    /// Resets the device and waits for it to become operational.
    pub async fn reset<D: DelayNs>(
        mut self,
        delay: &mut D,
    ) -> Result<Lsm6dso32<I, Uninitialised>, (Lsm6dso32<I, Uninitialised>, I::Error)>
    where
        I: AsyncRegisterInterface<AddressType = u8>,
    {
        let res = self
            .inner
            .ctrl_3_c()
            .modify_async(|r| r.set_sw_reset(true))
            .await;

        let rst = Lsm6dso32 {
            inner: self.inner,
            accel_full_scale: self.accel_full_scale,
            gyro_full_scale: self.gyro_full_scale,
            _state: PhantomData,
        };

        if let Err(err) = res {
            return Err((rst, err));
        }

        delay.delay_ms(30).await; // typical turn on time
        Ok(rst)
    }
}

impl<I> Lsm6dso32<I, Uninitialised>
where
    I: AsyncRegisterInterface<AddressType = u8>,
{
    /// Resets the device and waits for it to become operational. Initializes the sensor,
    /// enabling block data update and address auto-increment. Reads al necessary configuration
    /// values from the device.
    pub async fn init<D: DelayNs>(
        mut self,
        delay: &mut D,
    ) -> Result<Lsm6dso32<I, Initialised>, InitError<I, I::Error>> {
        self = match self.reset(delay).await {
            Ok(x) => x,
            Err((lsm, err)) => {
                return Err(InitError {
                    kind: InitErrorKind::BusError(err),
                    sensor: lsm,
                });
            }
        };

        // Check if the sensor responds with the correct WHOAMI value
        // If this fails, the device is either not an LSM6DSO32 or not
        // functioning correctly.
        match self.inner.who_am_i().read_async().await {
            Err(err) => {
                return Err(InitError {
                    kind: InitErrorKind::BusError(err),
                    sensor: self,
                });
            }
            Ok(whoami) => {
                if whoami.ident() != WHOAMI {
                    return Err(InitError {
                        kind: InitErrorKind::WhoamiMismatch,
                        sensor: self,
                    });
                }
            }
        }

        // Enable block data update and address auto-increment
        let res = self
            .inner
            .ctrl_3_c()
            .modify_async(|r| {
                r.set_bdu(true); // batch update
                r.set_if_inc(true); // auto increment
            })
            .await;
        if let Err(err) = res {
            return Err(InitError {
                kind: InitErrorKind::BusError(err),
                sensor: self,
            });
        }

        // read the full scale values from the device
        match self.inner.ctrl_1_xl().read_async().await {
            Err(err) => {
                return Err(InitError {
                    kind: InitErrorKind::BusError(err),
                    sensor: self,
                });
            }
            Ok(reg) => {
                self.accel_full_scale = reg.fs_xl();
            }
        }
        match self.inner.ctrl_2_g().read_async().await {
            Err(err) => {
                return Err(InitError {
                    kind: InitErrorKind::BusError(err),
                    sensor: self,
                });
            }
            Ok(ctrl_2_g) => {
                self.gyro_full_scale = ctrl_2_g.fs_g();
            }
        }

        Ok(Lsm6dso32 {
            inner: self.inner,
            accel_full_scale: self.accel_full_scale,
            gyro_full_scale: self.gyro_full_scale,
            _state: PhantomData,
        })
    }
}

/// Configuration for the INT2_CTRL register interrupt sources.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Int2Config {
    /// Enables COUNTER_BDR_IA interrupt on INT2 pin.
    pub cnt_bdr: bool,
    /// Enables FIFO full flag interrupt on INT2 pin.
    pub fifo_full: bool,
    /// Enables FIFO overrun interrupt on INT2 pin.
    pub fifo_ovr: bool,
    /// Enables FIFO threshold interrupt on INT2 pin.
    pub fifo_th: bool,
    /// Enables temperature sensor data-ready interrupt on INT2 pin.
    /// Can also trigger an IBI when using the MIPI I3C interface and `INT2_ON_INT1 = 1` in CTRL4_C (0x13).
    pub drdy_temp: bool,
    /// Enables gyroscope data-ready interrupt on INT2 pin.
    pub drdy_g: bool,
    /// Enables accelerometer data-ready interrupt on INT2 pin.
    pub drdy_xl: bool,
}

/// Configuration for the INT1_CTRL register interrupt sources.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Int1Config {
    /// Sends DEN_DRDY (Data Enable Stamped Data Ready) to INT1 pin.
    pub den_drdy_flag: bool,
    /// Enables COUNTER_BDR_IA interrupt on INT1 pin.
    pub cnt_bdr: bool,
    /// Enables FIFO full flag interrupt on INT1 pin.
    /// Can also trigger an IBI when using the MIPI I3C interface.
    pub fifo_full: bool,
    /// Enables FIFO overrun interrupt on INT1 pin. It can be also used to trigger an
    /// IBI when the MIPI I3CSM interface is used
    pub fifo_ovr: bool,
    /// Enables FIFO threshold interrupt on INT1 pin.
    /// Can also trigger an IBI when using the MIPI I3C interface.
    pub fifo_th: bool,
    /// Enables boot status indication on INT1 pin.
    pub boot: bool,
    /// Enables gyroscope data-ready interrupt on INT1 pin.
    /// Can also trigger an IBI when using the MIPI I3C interface.
    pub drdy_g: bool,
    /// Enables accelerometer data-ready interrupt on INT1 pin.
    /// Can also trigger an IBI when using the MIPI I3C interface.
    pub drdy_xl: bool,
}

impl<I> Lsm6dso32<I, Initialised>
where
    I: AsyncRegisterInterface<AddressType = u8>,
{
    /// Returns the current full-scale range of the accelerometer.
    pub const fn accel_full_scale(&self) -> AccelerometerFullScale {
        self.accel_full_scale
    }

    /// Returns the current full-scale range of the gyroscope.
    pub const fn gyro_full_scale(&self) -> GyroscopeFullScale {
        self.gyro_full_scale
    }

    /// Reads the current acceleration values (x, y, z) and converts them to g.
    pub async fn read_acceleration(&mut self) -> Result<Acceleration, I::Error> {
        let data = self.inner.out_a().read_async().await?;
        let x = data.linear_acceleration_sensor_x();
        let y = data.linear_acceleration_sensor_y();
        let z = data.linear_acceleration_sensor_z();
        let raw = AccelerationRaw { x, y, z };
        Ok(Acceleration::from_raw(raw, self.accel_full_scale))
    }

    /// Reads the current angular rate (x, y, z) and converts them to dps.
    pub async fn read_angular_rate(&mut self) -> Result<AngularRate, I::Error> {
        let data = self.inner.out_g().read_async().await?;
        let x = data.angular_rate_sensor_x();
        let y = data.angular_rate_sensor_y();
        let z = data.angular_rate_sensor_z();
        let raw = AngularRateRaw { x, y, z };
        Ok(AngularRate::from_raw(raw, self.gyro_full_scale))
    }

    /// Reads the current temperature and converts it to °C.
    pub async fn read_temperature(&mut self) -> Result<Temperature, I::Error> {
        let data = self.inner.out_temp().read_async().await?;
        let value = data.temp();
        Ok(Temperature::from_raw(TemperatureRaw { value }))
    }

    /// Reads the current acceleration and angular rate values.
    pub async fn read_accelerometer_and_gyroscope(
        &mut self,
    ) -> Result<(Acceleration, AngularRate), I::Error> {
        let data = self.inner.out_ga().read_async().await?;

        let accel = Acceleration::from_raw(
            AccelerationRaw {
                x: data.linear_acceleration_sensor_x(),
                y: data.linear_acceleration_sensor_y(),
                z: data.linear_acceleration_sensor_z(),
            },
            self.accel_full_scale,
        );
        let gyro = AngularRate::from_raw(
            AngularRateRaw {
                x: data.angular_rate_sensor_x(),
                y: data.angular_rate_sensor_y(),
                z: data.angular_rate_sensor_z(),
            },
            self.gyro_full_scale,
        );

        Ok((accel, gyro))
    }

    /// Reads the status register
    pub async fn data_ready(&mut self) -> Result<StatusReg, I::Error> {
        self.inner.status_reg().read_async().await
    }

    /// Sets the output data rate and full-scale range for the accelerometer.
    pub async fn set_accelerometer_odr_and_full_scale(
        &mut self,
        odr: Option<AccelerometerOdr>,
        full_scale: Option<AccelerometerFullScale>,
    ) -> Result<(), I::Error> {
        self.inner
            .ctrl_1_xl()
            .modify_async(|r| {
                if let Some(odr) = odr {
                    r.set_odr_xl(odr);
                }
                if let Some(full_scale) = full_scale {
                    r.set_fs_xl(full_scale);
                }
            })
            .await?;
        if let Some(full_scale) = full_scale {
            self.accel_full_scale = full_scale;
        }
        Ok(())
    }

    /// Sets the output data rate and full-scale range for the gyroscope.
    pub async fn set_gyroscope_odr_and_full_scale(
        &mut self,
        odr: Option<GyroscopeOdr>,
        full_scale: Option<GyroscopeFullScale>,
    ) -> Result<(), I::Error> {
        self.inner
            .ctrl_2_g()
            .modify_async(|r| {
                if let Some(odr) = odr {
                    r.set_odr_g(odr);
                }
                if let Some(full_scale) = full_scale {
                    r.set_fs_g(full_scale);
                }
            })
            .await?;
        if let Some(full_scale) = full_scale {
            self.gyro_full_scale = full_scale;
        }
        Ok(())
    }

    /// Configures the FIFO mode.
    pub async fn set_fifo_mode(&mut self, mode: FifoMode) -> Result<(), I::Error> {
        self.inner
            .fifo_ctrl_4()
            .modify_async(|r| r.set_fifo_mode(mode))
            .await
    }

    /// Configures the FIFO settings by setting the watermark and whether to stop on watermark.
    pub async fn configure_fifo(
        &mut self,
        watermark: u16,
        stop_on_watermark: bool,
    ) -> Result<(), I::Error> {
        self.inner_mut()
            .fifo_ctrl_1_and_2()
            .modify_async(|reg| {
                reg.set_watermark(watermark);
                reg.set_stop_on_wtm(stop_on_watermark);
            })
            .await
    }

    /// Configure the interrupts for the sensor
    pub async fn configure_interrupts(
        &mut self,
        int1: Option<Int1Config>,
        int2: Option<Int2Config>,
    ) -> Result<(), I::Error> {
        if let Some(int1) = int1 {
            self.inner
                .int_1_ctrl()
                .write_async(|r| {
                    r.set_int1_cnt_bdr(int1.cnt_bdr);
                    r.set_int1_fifo_full(int1.fifo_full);
                    r.set_int1_fifo_ovr(int1.fifo_ovr);
                    r.set_int1_fifo_th(int1.fifo_th);
                    r.set_int1_boot(int1.boot);
                    r.set_int1_drdy_g(int1.drdy_g);
                    r.set_int1_drdy_xl(int1.drdy_xl);
                })
                .await?;
        }
        if let Some(int2) = int2 {
            self.inner
                .int_2_ctrl()
                .write_async(|r| {
                    r.set_int2_cnt_bdr(int2.cnt_bdr);
                    r.set_int2_fifo_full(int2.fifo_full);
                    r.set_int2_fifo_ovr(int2.fifo_ovr);
                    r.set_int2_fifo_th(int2.fifo_th);
                    r.set_int2_drdy_temp(int2.drdy_temp);
                    r.set_int2_drdy_g(int2.drdy_g);
                    r.set_int2_drdy_xl(int2.drdy_xl);
                })
                .await?;
        }
        Ok(())
    }

    /// Reads the current number of unread entries in the FIFO buffer.
    pub async fn read_fifo_level(&mut self) -> Result<u16, I::Error> {
        let status1 = self.inner.fifo_status_1().read_async().await?;
        let status2 = self.inner.fifo_status_2().read_async().await?;
        let diff_fifo = ((status2.diff_fifo() as u16) << 8) | (status1.diff_fifo() as u16);
        Ok(diff_fifo)
    }

    /// Sets the batch data rates for accelerometer, gyroscope, temperature, and timestamp in the FIFO.
    pub async fn set_fifo_batch_data_rates(
        &mut self,
        acc_odr: Option<AccelBatchDataRate>,
        gyr_odr: Option<GyroBatchDataRate>,
        temp_batch_data_rate: Option<TempBatchDataRate>,
        dec_ts_batch: Option<DecTsBatch>,
    ) -> Result<(), I::Error> {
        self.inner
            .fifo_ctrl_4()
            .modify_async(|r| {
                if let Some(temp_batch_data_rate) = temp_batch_data_rate {
                    r.set_odr_t_batch(temp_batch_data_rate);
                }
                if let Some(dec_ts_batch) = dec_ts_batch {
                    r.set_dec_ts_batch(dec_ts_batch);
                }
            })
            .await?;
        self.inner
            .fifo_ctrl_3()
            .modify_async(|r| {
                if let Some(gyr_odr) = gyr_odr {
                    r.set_bdr_gy(gyr_odr);
                }
                if let Some(acc_odr) = acc_odr {
                    r.set_bdr_xl(acc_odr);
                }
            })
            .await?;
        Ok(())
    }

    /// Reads up to 512 FIFO data frames from the sensor into the provided buffer.
    ///
    /// Each frame is 7 bytes (1 byte tag + 6 bytes payload). This function performs a
    /// bulk register read starting at address `0x78` using the sensor interface, and
    /// casts the result directly into `FifoDataOut` entries via `from_raw_parts_mut`.
    ///
    /// The caller must ensure that there are enough elements in the fifo to read and
    /// must validate the `tag_sensor` field of each entry before using the data.
    pub async fn read_multiple_fifo_data(
        &mut self,
        data: &mut [FifoDataOut],
    ) -> Result<(), I::Error> {
        const MAX_FIFO_DATA: usize = 512;
        let max_count = min(MAX_FIFO_DATA, data.len());

        // SAFETY:
        // - FifoDataOut is #[repr(C)] with [u8; 7] inside (no padding)
        // - Alignment is 1
        // - Total size is exactly max_count * 7 bytes
        let buf = unsafe {
            core::slice::from_raw_parts_mut(
                data.as_mut_ptr() as *mut u8,
                max_count * size_of::<FifoDataOut>(),
            )
        };

        self.inner
            .interface
            .read_register(0x78, (buf.len() * 8) as u32, buf)
            .await?;

        Ok(())
    }
}
