// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![allow(dead_code)]
#![no_std]

use embassy_stm32::i2c::{Error, I2c, mode::Master};
use embassy_stm32::mode::Async;
use embassy_time::Timer;

// MAX31889 sensor abstraction.

pub struct MAX31889<'a> {
    i2c: I2c<'a, Async, Master>,
    address: u8,
}

impl<'a> MAX31889<'a> {
    pub fn new(i2c: I2c<'a, Async, Master>, address: u8) -> Self {
        MAX31889 { i2c, address }
    }

    pub async fn combined_temperature_read(&mut self) -> Result<f32, Error> {
        let control_reg = 0x14;
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[control_reg], &mut buf)
            .await?;
        buf[0] |= 0b1; // set the control bit to start measurement
        self.i2c.write(self.address, &[control_reg, buf[0]]).await?;
        let fifo_reg = 0x08;
        let mut res_buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[fifo_reg], &mut res_buf)
            .await?;
        let raw: i16 = (res_buf[0] as i16) << 8 | res_buf[1] as i16;
        Ok(raw as f32 * 0.005)
    }

    pub async fn trigger_measurement(&mut self) -> Result<(), Error> {
        let control_reg = 0x14;
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[control_reg], &mut buf)
            .await?;
        buf[0] |= 0b1; // set the control bit to start measurement
        self.i2c.write(self.address, &[control_reg, buf[0]]).await
    }

    pub async fn read_fifo(&mut self) -> Result<f32, Error> {
        let fifo_reg = 0x08;
        let mut res_buf = [0u8; 2];
        Timer::after_millis(10).await;
        self.i2c
            .write_read(self.address, &[fifo_reg], &mut res_buf)
            .await?;
        let raw: u16 = (res_buf[0] as u16) << 8 | res_buf[1] as u16;
        Ok(raw as f32 * 0.005)
    }
}
