device_driver::create_device!(
    device_name: Lsm6dso32device,
    dsl: {
        config {
            type DefaultRegisterAccess = RO;
            type DefaultFieldAccess = RW;
            type DefaultBufferAccess = RW;
            type DefaultByteOrder = LE;
            type DefaultBitOrder = LSB0;
            type RegisterAddressType = u8;
            type NameWordBoundaries = [
                Underscore, Hyphen, Space, LowerUpper,
                //UpperDigit, DigitUpper, DigitLower,
                //LowerDigit, Acronym,
            ];
            type DefmtFeature = "defmt-03";
        }

        /// Enable embedded functions register (r/w)
        register FUNC_CFG_ACCESS {
            const ADDRESS = 0x01; // 01h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Enable access to the embedded functions configuration registers.
            FUNC_CFG_ACCESS: bool = 7,

            /// Enable access to the sensor hub (I²C master) registers.
            SHUB_REG_ACCESS: bool = 6,
        },

        register PIN_CTRL  {
            const ADDRESS = 0x02; // 01h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0b00111111;

            /// Enable pull-up on SDO pin
            /// (0: SDO pin pull-up disconnected (default); 1: SDO pin with pull-up)
            SDO_PU_EN: bool = 6,
        },

        /// FIFO control register 1 (r/w)
        register FIFO_CTRL1 {
            const ADDRESS = 0x07; // 07h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// FIFO watermark threshold, in conjunction with WTM8 in FIFO_CTRL2 (08h)
            /// 1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO
            /// Watermark flag rises when the number of bytes written in the FIFO is greater than or
            /// equal to the threshold level.
            WTM7_0: uint = 0..=7, // TODO find a prettier way to handle split values
        },

        /// FIFO control register 2 (r/w)
        register FIFO_CTRL2 {
            const ADDRESS = 0x08; // 08h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Sensing chain FIFO stop values memorization at threshold level
            /// (0: FIFO depth is not limited (default);
            /// 1: FIFO depth is limited to threshold level, defined in FIFO_CTRL1 (07h) and
            /// FIFO_CTRL2 (08h))
            ///
            /// Note: This bit is effective if the FIFO_COMPR_EN bit of EMB_FUNC_EN_B (05h) is set to 1.
            STOP_ON_WTM: bool = 7,

            /// Enables/Disables compression algorithm runtime
            FIFO_COMPR_RT_EN: bool = 6,

            /// Enables ODR CHANGE virtual sensor to be batched in FIFO
            ODRCHG_EN: bool = 4,

            /// This field configures the compression algorithm to write non-compressed data at
            /// each rate.
            /// (0: Non-compressed data writing is not forced;
            /// 1: Non-compressed data every 8 batch data rate;
            /// 2: Non-compressed data every 16 batch data rate;
            /// 3: Non-compressed data every 32 batch data rate)
            UNCOPTR_RATE: uint = 1..=2,

            /// FIFO watermark threshold, in conjunction with WTM_FIFO[7:0] in FIFO_CTRL1
            /// (07h)
            /// 1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO
            /// Watermark flag rises when the number of bytes written in the FIFO is greater than
            /// or equal to the threshold leve
            WTM8: uint = 0..1, // TODO find a prettier way to handle split values

        },

        /// FIFO Control Registers 1 and 2 Compound Register (r/w)
        register FIFO_CTRL1_AND_2 {
            const ADDRESS = 0x07; // same as FIFO_CTRL1
            const SIZE_BITS = 16;
            type Access = RW;
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound

            /// FIFO watermark threshold (9-bit value):
            /// Lower 8 bits from FIFO_CTRL1 (WTM7_0) and 1 bit from FIFO_CTRL2 (WTM8)
            Watermark: uint = 0..=8,

            /// Configures the compression algorithm to write non-compressed data at each rate.
            /// (bits 1..=2 in FIFO_CTRL2)
            UNCOPTR_RATE: uint = 9..=10,

            /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
            /// (bit 4 in FIFO_CTRL2)
            ODRCHG_EN: bool = 12,

            /// Enables/Disables compression algorithm runtime.
            /// (bit 6 in FIFO_CTRL2)
            FIFO_COMPR_RT_EN: bool = 14,

            /// Sensing chain FIFO stop values memorization at threshold level.
            /// (bit 7 in FIFO_CTRL2)
            STOP_ON_WTM: bool = 15,
        },


        /// FIFO control register 3 (r/w)
        register FIFO_CTRL3 {
            const ADDRESS = 0x09; // 09h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;


            /// Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.
            BDR_GY: uint as enum GyroBatchDataRate {
                /// Gyro not batched in FIFO (default);
                NotBatched = 0b0000,
                /// 12.5 Hz;
                Hz12_5 = 0b0001,
                /// 26 Hz;
                Hz26 = 0b0010,
                /// 52 Hz;
                Hz52 = 0b0011,
                /// 104 Hz;
                Hz104 = 0b0100,
                /// 208 Hz;
                Hz208 = 0b0101,
                /// 417 Hz;
                Hz417 = 0b0110,
                /// 833 Hz;
                Hz833 = 0b0111,
                /// 1667 Hz;
                Hz1667 = 0b1000,
                /// 3333 Hz;
                Hz3333 = 0b1001,
                /// 6667 Hz;
                Hz6667 = 0b1010,
                /// 6.5 Hz;
                Hz6_5 = 0b1011,
                /// Not allowed
                NotAllowed = catch_all,
            } = 4..=7,

            /// Selects Batching Data Rate (writing frequency in FIFO) for accelerometer data.
            BDR_XL: uint as enum AccelBatchDataRate {
                /// Accel not batched in FIFO (default);
                NotBatched = 0b0000,
                /// 12.5 Hz;
                Hz12_5 = 0b0001,
                /// 26 Hz;
                Hz26 = 0b0010,
                /// 52 Hz;
                Hz52 = 0b0011,
                /// 104 Hz;
                Hz104 = 0b0100,
                /// 208 Hz;
                Hz208 = 0b0101,
                /// 417 Hz;
                Hz417 = 0b0110,
                /// 833 Hz;
                Hz833 = 0b0111,
                /// 1667 Hz;
                Hz1667 = 0b1000,
                /// 3333 Hz;
                Hz3333 = 0b1001,
                /// 6667 Hz;
                Hz6667 = 0b1010,
                /// 6.5 Hz;
                Hz6_5 = 0b1011,
                /// Not allowed
                NotAllowed = catch_all,
            } = 0..=3,
        },

        /// FIFO control register 4 (r/w)
        register FIFO_CTRL4 {
            const ADDRESS = 0x0A; // 0Ah
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Selects decimation for timestamp batching in FIFO. Writing rate will be the maximum
            /// rate between XL and GYRO BDR divided by decimation decoder.
            DEC_TS_BATCH: uint as enum DecTsBatch {
                /// 00: Timestamp not batched in FIFO (default);
                NotBatched = 0b00,
                /// Decimation 1: max(BDR_XL[Hz],BDR_GY[Hz]) [Hz];
                Dec1 = 0b01,
                /// Decimation 8: max(BDR_XL[Hz],BDR_GY[Hz]) [Hz]/8;
                Dec8 = 0b10,
                /// Decimation 32: max(BDR_XL[Hz],BDR_GY[Hz]) [Hz]/32;
                Dec32 = 0b11,
            } = 6..=7,

            /// Selects batching data rate (writing frequency in FIFO) for temperature data
            ODR_T_BATCH: uint as enum TempBatchDataRate {
                /// 00: Temperature not batched in FIFO (default);
                NotBatched = 0b00,
                /// 01: 1 Hz;
                Hz1 = 0b01,
                /// 10: 12.5 Hz;
                Hz12_5 = 0b10,
                /// 11: 52 Hz;
                Hz52 = 0b11,
            } = 4..=5,


            /// FIFO mode selection
            FIFO_MODE: uint as enum FifoMode {
                /// Bypass mode: FIFO disabled;
                Bypass = 0b000,
                /// FIFO mode: stops collecting data when FIFO is full;
                Fifo = 0b001,
                /// Reserved;
                Reserved = catch_all,
                /// Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then FIFO mode;
                ContinuousToFifo = 0b011,
                /// Bypass-to-Continuous mode: Bypass mode until trigger is deasserted, then Continuous mode;
                BypassToContinuous = 0b100,
                /// Continuous mode: if the FIFO is full, the new sample overwrites the older one;
                Continuous = 0b110,
                /// Bypass-to-FIFO mode: Bypass mode until trigger is deasserted, then FIFO mode.
                BypassToFifo = 0b111,
            } = 0..=2,

        },

        /// Counter batch data rate register 1 (r/w)
        register COUNTER_BDR_REG1  {
            const ADDRESS = 0x0B; // 0Bh
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Enables pulsed data-ready mode
            /// (0: Data-ready latched mode (returns to 0 only after an interface reading)
            /// (default);
            /// 1: Data-ready pulsed mode (the data ready pulses are 75 μs long)
            Dataready_pulsed: bool = 7,

            /// Resets the internal counter of batching events for a single sensor.
            /// This bit is automatically reset to zero if it was set to ‘1’.
            RST_COUNTER_BDR: bool = 6,

            /// Selects the trigger for the internal counter of batching events between XL and
            /// gyro.
            /// (0: XL batching event;
            /// 1: GYRO batching event)
            TRIG_COUNTER_BDR: bool = 5,

            /// In conjunction with CNT_BDR_TH_[7:0] in COUNTER_BDR_REG2 (0Ch),
            /// sets the threshold for the internal counter of batching events. When this
            /// counter reaches the threshold, the counter is reset and the
            /// COUNTER_BDR_IA flag in FIFO_STATUS2 (3Bh) is set to ‘1’.
            CNT_BDR_TH_10_8: uint = 0..=2, // TODO find a prettier way to handle split values
        },

        /// Counter batch data rate register 2 (r/w)
        register COUNTER_BDR_REG2  {
            const ADDRESS = 0x0C; // 0Ch
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// In conjunction with CNT_BDR_TH_[10:8] in COUNTER_BDR_REG1 (0Bh), sets
            /// the threshold for the internal counter of batching events. When this counter reaches
            /// the threshold, the counter is reset and the COUNTER_BDR_IA flag in
            /// FIFO_STATUS2 (3Bh) is set to ‘1’.
            CNT_BDR_TH_7_0: uint = 0..=7,
        },

        /// INT1 pin control register (r/w)
        register INT1_CTRL {
            const ADDRESS = 0x0D; // 0Dh
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Sends DEN_DRDY (DEN stamped on Sensor Data flag) to INT1 pin
            DEN_DRDY_flag: bool = 7,

            /// Enables COUNTER_BDR_IA interrupt on INT1
            INT1_CNT_BDR: bool = 6,

            /// Enables FIFO full flag interrupt on INT1 pin. It can be also used to
            /// trigger an IBI when the MIPI I3CSM interface is used.
            INT1_FIFO_FULL: bool = 5,

            /// Enables FIFO overrun interrupt on INT1 pin. It can be also used to trigger an
            /// IBI when the MIPI I3CSM interface is used.
            INT1_FIFO_OVR: bool = 4,

            /// Enables FIFO threshold interrupt on INT1 pin. It can be also used to trigger
            /// an IBI when the MIPI I3CSM interface is used.
            INT1_FIFO_TH: bool = 3,

            /// Enables boot status on INT1 pin
            INT1_BOOT: bool = 2,

            /// Enables gyroscope data-ready interrupt on INT1 pin. It can be also used to
            /// trigger an IBI when the MIPI I3CSM interface is used.
            INT1_DRDY_G: bool = 1,

            /// Enables accelerometer data-ready interrupt on INT1 pin. It can be also used
            /// to trigger an IBI when the MIPI I3CSM interface is used.
            INT1_DRDY_XL: bool = 0,
        },

        /// INT2 pin control register (r/w)
        register INT2_CTRL {
            const ADDRESS = 0x0E; // 0Eh
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Enables COUNTER_BDR_IA interrupt on INT2
            INT2_CNT_BDR: bool = 6,

            /// Enables FIFO full flag interrupt on INT2 pin
            INT2_FIFO_FULL: bool = 5,

            /// Enables FIFO overrun interrupt on INT2 pin
            INT2_FIFO_OVR: bool = 4,

            /// Enables FIFO threshold interrupt on INT2 pin
            INT2_FIFO_TH: bool = 3,

            /// Enables temperature sensor data-ready interrupt on INT2 pin. It
            /// can be also used to trigger an IBI when the MIPI I3CSM interface is used and
            /// INT2_ON_INT1 = ‘1’ in CTRL4_C (13h).
            INT2_DRDY_TEMP: bool = 2,

            /// Gyroscope data-ready interrupt on INT2 pin
            INT2_DRDY_G: bool = 1,

            /// Accelerometer data-ready interrupt on INT2 pin
            INT2_DRDY_XL: bool = 0,

        },

        /// WHO_AM_I register (r). This is a read-only register. Its value is fixed at 6Ch.
        register WHO_AM_I {
            const ADDRESS = 0x0F; // 0Fh
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0b01101100;

            /// identification
            ident: uint = 0..=7,
        },

        /// Accelerometer control register 1 (r/w)
        register CTRL1_XL {
            const ADDRESS = 0x10; // 10h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;
            /// Accelerometer ODR selection [Hz] when
            /// XL_HM_MODE = 1 in CTRL6_C (15h)
            ODR_XL: uint as enum AccelerometerODR {
                /// Power-down mode (default);
                PowerDown = 0b0000,
                /// 12.5 Hz (low power only)
                Hz12_5 = 0b0001,
                /// 26 Hz (low power only)
                Hz26 = 0b0010,
                /// 52 Hz (low power only)
                Hz52 = 0b0011,
                /// 104 Hz (normal mode)
                Hz104 = 0b0100,
                /// 208 Hz (normal mode)
                Hz208 = 0b0101,
                /// 416 Hz (high performance)
                Hz416 = 0b0110,
                /// 833 Hz (high performance)
                Hz833 = 0b0111,
                /// 1.66 kHz (high performance)
                Hz1667 = 0b1000,
                /// 3.33 kHz (high performance)
                Hz3333 = 0b1001,
                /// 6.66 kHz (high performance)
                Hz6667 = 0b1010,
                /// 1.6 Hz (low power only)
                /// If XL_HM_MODE = 0, then 12.5 Hz instead.
                Hz1_6 = 0b1011,
                /// Not allowed
                NotAllowed = catch_all,
            } = 4..=7,



            /// Accelerometer full-scale selection
            FS_XL: uint as enum AccelerometerFullScale {
                /// ±4g (default)
                G4 = 0b00,
                /// ±32g
                G32 = 0b01,
                /// ±8g
                G8 = 0b10,
                /// ±16g
                G16 = 0b11,
            } = 2..=3,

            /// Accelerometer high-resolution selection
            /// (0: output from first stage digital filtering selected (default);
            /// 1: output from LPF2 second filtering stage selected)
            LPF2_XL_EN: bool = 1,
        },

        /// Gyroscope control register 2 (r/w)
        register CTRL2_G {
            const ADDRESS = 0x11; // 11h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// ODR [Hz] when G_HM_MODE = 1 in CTRL7_G (16h)
            ODR_G: uint as enum GyroscopeODR {
                /// Power-down mode (default);
                PowerDown = 0b0000,
                /// 12.5 Hz (low power only)
                Hz12_5 = 0b0001,
                /// 26 Hz (low power only)
                Hz26 = 0b0010,
                /// 52 Hz (low power only)
                Hz52 = 0b0011,
                /// 104 Hz (normal mode)
                Hz104 = 0b0100,
                /// 208 Hz (normal mode)
                Hz208 = 0b0101,
                /// 416 Hz (high performance)
                Hz416 = 0b0110,
                /// 833 Hz (high performance)
                Hz833 = 0b0111,
                /// 1.66 kHz (high performance)
                Hz1667 = 0b1000,
                /// 3.33 kHz (high performance)
                Hz3333 = 0b1001,
                /// 6.66 kHz (high performance)
                Hz6667 = 0b1010,
                /// 1.6 Hz (low power only)
                /// If XL_HM_MODE = 0, then 12.5 Hz instead.
                Hz1_6 = 0b1011,
                /// Not allowed
                NotAllowed = catch_all,
            } = 4..=7,


            /// Gyroscope chain full-scale selection
            FS_G: uint as enum GyroscopeFullScale {
                /// ±250 dps
                Dps250 = 0b00,
                /// ±500 dps
                Dps500 = 0b01,
                /// ±1000 dps
                Dps1000 = 0b10,
                /// ±2000 dps
                Dps2000 = 0b11,
            } = 2..=3,

            /// Accelerometer high-resolution selection
            /// (0: output from first stage digital filtering selected (default);
            /// 1: output from LPF2 second filtering stage selected)
            FS_125: bool = 1,
        },

        /// Control register 3 (r/w)
        register CTRL3_C {
            const ADDRESS = 0x12; // 12h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0b00000100;

            /// Reboots memory content. Default value: 0
            /// (0: normal mode; 1: reboot memory content)
            /// This bit is automatically cleared.
            BOOT: bool = 7,

            /// Block Data Update. Default value: 0
            /// (0: continuous update;
            /// 1: output registers are not updated until MSB and LSB have been read)
            BDU: bool = 6,

            /// Interrupt activation level. Default value: 0
            /// (0: interrupt output pins active high; 1: interrupt output pins active low)
            H_LACTIVE: bool = 5,

            /// Push-pull/open-drain selection on INT1 and INT2 pins. Default value: 0
            /// (0: push-pull mode; 1: open-drain mode)
            PP_OD: bool = 4,

            /// SPI Serial Interface Mode selection. Default value: 0
            /// (0: 4-wire interface; 1: 3-wire interface)
            SIM: bool = 3,

            /// Register address automatically incremented during a multiple byte access with a
            /// serial interface (I²C or SPI). Default value: 1
            /// (0: disabled; 1: enabled)
            IF_INC: bool = 2,

            /// Software reset. Default value: 0
            /// (0: normal mode; 1: reset device)
            /// This bit is automatically cleared.
            SW_RESET: bool = 0,
        },

        /// Control register 4 (r/w)
        register CTRL4_C {
            const ADDRESS = 0x13; // 13h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Enables gyroscope Sleep mode. Default value:0
            /// (0: disabled; 1: enabled)
            SLEEP_G: bool = 6,

            /// All interrupt signals available on INT1 pin enable. Default value: 0
            /// (0: interrupt signals divided between INT1 and INT2 pins;
            /// 1: all interrupt signals in logic or on INT1 pin)
            INT2_on_INT1: bool = 5,

            /// Enables data available
            /// (0: disabled;
            /// 1: mask DRDY on pin (both XL & Gyro) until filter settling ends (XL and Gyro
            /// independently masked).
            DRDY_MASK: bool = 3,

            /// Disables I²C interface. Default value: 0
            /// (0: SPI, I²C and MIPI I3CSM interfaces enabled (default); 1: I²C interface disabled)
            I2C_disable: bool = 2,

            /// Enables gyroscope digital LPF1; the bandwidth can be selected through
            /// FTYPE [2:0] in CTRL6_C (15h).
            /// (0: disabled; 1: enabled)
            LPF1_SEL_G: bool = 1,
        },

        /// Control register 5 (r/w)
        register CTRL5_C {
            const ADDRESS = 0x14; // 14h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Accelerometer ultra-low-power mode enable(1). Default value: 0
            /// (0: Ultra-low-power mode disabled; 1: Ultra-low-power mode enabled)
            XL_ULP_EN: bool = 7,

            /// Circular burst-mode (rounding) read from the output registers. Default value: 00
            ROUNDING: uint as enum CircularBurstMode {
                /// no rounding
                NoRounding = 0b00,
                /// accelerometer only
                AccelerometerOnly = 0b01,
                /// gyroscope only
                GyroscopeOnly = 0b10,
                /// gyroscope and accelerometer
                GyroscopeAndAccelerometer = 0b11,
            } = 5 ..=6,

            /// Angular rate sensor self-test enable. Default value: 00
            /// (00: Self-test disabled; Other: refer to Table 55)
            ST_G: uint as enum GyroscopeSelfTest {
                /// normal mode
                NormalMode = 0b00,
                /// Positive sign self-test
                PositiveSignSelfTest = 0b01,
                /// Not allowed
                NotAllowed = 0b10,
                /// Negative sign self-test
                NegativeSignSelfTest = 0b11,
            } = 2..=3,

            /// Linear acceleration sensor self-test enable. Default value: 00
            /// (00: Self-test disabled; Other: refer to Table 56)
            ST_XL: uint as enum AccelerometerSelfTest {
               /// normal mode
                NormalMode = 0b00,
                /// Positive sign self-test
                PositiveSignSelfTest = 0b01,
                /// Not allowed
                NotAllowed = 0b10,
                /// Negative sign self-test
                NegativeSignSelfTest = 0b11,
            } = 0..=1,

        },


            /// Control register 6 (r/w)
        register CTRL6_C {
            const ADDRESS = 0x15; // 15h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// DEN data edge-sensitive trigger enable.
            /// DEN data level-sensitive trigger enable.
            /// DEN level-sensitive latched enable.
            DEN: uint = 5..=7,

            /// High-performance operating mode disable for accelerometer. Default value: 0
            /// (0: high-performance operating mode enabled;
            /// 1: high-performance operating mode disabled)
            XL_HM_MODE: bool = 4,

            /// Weight of XL user offset bits of registers X_OFS_USR (73h), Y_OFS_USR (74h),
            /// Z_OFS_USR (75h)
            /// 0 = 2-10 g/LSB
            /// 1 = 2-6 g/LSB
            USR_OFF_W: bool = 3,


            /// Gyroscope's low-pass filter (LPF1) bandwidth selection
            /// Table 60 shows the selectable bandwidth values.
            FTYPE: uint = 0 ..=2,
        },

        /// Control register 7 (r/w)
        register CTRL7_G {
            const ADDRESS = 0x16; // 16h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Disables high-performance operating mode for gyroscope. Default: 0
            /// (0: high-performance operating mode enabled;
            /// 1: high-performance operating mode disabled)
            G_HM_MODE: bool = 7,

            /// Enables gyroscope digital high-pass filter. The filter is enabled only if the gyro is
            /// in HP mode. Default: 0
            /// (0: HPF disabled; 1: HPF enabled)
            HP_EN_G: bool = 6,

            /// Gyroscope digital HP filter cutoff selection. Default value: 00
            HPM_G: uint as enum GyroscopeHighPassFilterCutoff {
                /// 16 mHz
                mHz16 = 0b00,
                /// 65 mHz
                mHz65 = 0b01,
                /// 260 mHz
                mHz260 = 0b10,
                /// 1_04 Hz
                Hz1_04 = 0b11,
            } = 4..=5,

            /// Enables accelerometer user offset correction block; it's valid for the low-pass
            /// path - see Figure 17: Accelerometer composite filter. Default value: 0
            /// (0: accelerometer user offset correction block bypassed;
            /// 1: accelerometer user offset correction block enabled)
            USR_OFF_ON_OUT: bool = 1,

        },


        /// Control register 8 (r/w)
        register CTRL8_XL {
            const ADDRESS = 0x17; // 17h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Accelerometer LPF2 and HP filter configuration and cutoff setting.
            /// Note: The cutoff frequency depends on the accelerometer ODR.
            /// Please refer to the datasheet (e.g., Table 65) for details.
            HPCF_XL: uint = 5..=7,

            /// Enables accelerometer high-pass filter reference mode (valid for high-pass path -
            /// HP_SLOPE_XL_EN bit must be ‘1’). Default value: 0
            /// (0: disabled, 1: enabled)
            HP_REF_MODE_XL: bool = 4,

            /// Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the
            /// second samples after writing this bit. Active only during device exit from power-
            /// down mode. Default value: 0
            /// (0: disabled, 1: enabled)
            FASTSETTL_MODE_XL: bool = 3,

            /// Accelerometer slope filter / high-pass filter selection. Refer to Figure 21.
            /// (0: low pass, 1: high pass)
            HP_SLOPE_XL_EN: bool = 2,

            /// LPF2 on 6D function selection. Refer to Figure 21. Default value: 0
            /// (0: ODR/2 low-pass filtered data sent to 6D interrupt function;
            /// 1: LPF2 output data sent to 6D interrupt function)
            LOW_PASS_ON_6D: bool = 0,
        },

        /// Control register 9 (r/w)
        register CTRL9_XL {
            const ADDRESS = 0x18; // 18h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0b11100000;

            /// DEN value stored in LSB of X-axis. Default value: 1
            /// (0: DEN not stored in X-axis LSB; 1: DEN stored in X-axis LSB)
            DEN_X: bool = 7,

            /// DEN value stored in LSB of Y-axis. Default value: 1
            /// (0: DEN not stored in Y-axis LSB; 1: DEN stored in Y-axis LSB)
            DEN_Y: bool = 6,

            /// DEN value stored in LSB of Z-axis. Default value: 1
            /// (0: DEN not stored in Z-axis LSB; 1: DEN stored in Z-axis LSB)
            DEN_Z: bool = 5,

            /// DEN stamping sensor selection. Default value: 0
            /// (0: DEN pin info stamped in the gyroscope axis selected by bits [7:5];
            /// 1: DEN pin info stamped in the accelerometer axis selected by bits [7:5])
            DEN_XL_G: bool = 4,

            /// Extends DEN functionality to accelerometer sensor. Default value: 0
            /// (0: disabled; 1: enabled)
            DEN_XL_EN: bool = 3,

            /// DEN active level configuration. Default value: 0
            /// (0: active low; 1: active high)
            DEN_LH: bool = 2,

            /// Disables MIPI I3CSM communication protocol(1)
            /// (0: SPI, I²C, MIPI I3CSM interfaces enabled (default);
            /// 1: MIPI I3CSM interface disabled)
            I3C_disable: bool = 1,
        },

        /// Control register 10 (r/w)
        register CTRL10_C {
            const ADDRESS = 0x19; // 19h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Enables timestamp counter. default value: 0
            /// (0: disabled; 1: enabled)
            /// The counter is readable in TIMESTAMP0 (40h), TIMESTAMP1 (41h),
            /// TIMESTAMP2 (42h), and TIMESTAMP3 (43h).
            TIMESTAMP_EN: bool = 5,
        },

        /// Source register for all interrupts (r)
        register ALL_INT_SRC {
            const ADDRESS = 0x1A; // 1Ah
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Alerts timestamp overflow within 6.4 ms
            TIMESTAMP_ENDCOUNT: bool = 7,

            /// Detects change event in activity/inactivity status. Default value: 0
            /// (0: change status not detected; 1: change status detected)
            SLEEP_CHANGE_IA: bool = 5,

            /// Interrupt active for change in position of portrait, landscape, face-up,
            /// face-down. Default value: 0
            /// (0: change in position not detected; 1: change in position detected)
            D6D_IA: bool = 4,

            /// Double-tap event status. Default value: 0
            /// (0:event not detected, 1: event detected)
            DOUBLE_TAP: bool = 3,

            /// Single-tap event status. Default value: 0
            /// (0: event not detected, 1: event detected)
            SINGLE_TAP: bool = 2,

            /// Wake-up event status. Default value: 0
            /// (0: event not detected, 1: event detected)
            WU_IA: bool = 1,

            /// Free-fall event status. Default value: 0
            /// (0: event not detected, 1: event detected)
            FF_IA: bool = 0,
        },

        /// Wake-up interrupt source register (r)
        register WAKE_UP_SRC {
            const ADDRESS = 0x1B; // 1Bh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Detects change event in activity/inactivity status. Default value: 0
            /// (0: change status not detected; 1: change status detected)
            SLEEP_CHANGE_IA: bool = 6,

            /// Free-fall event detection status. Default value: 0
            /// (0: free-fall event not detected; 1: free-fall event detected)
            FF_IA: bool = 5,

            /// Sleep status bit. Default value: 0
            /// (0: Activity status; 1: Inactivity status)
            SLEEP_STATE: bool = 4,

            /// Wakeup event detection status. Default value: 0
            /// (0: wakeup event not detected; 1: wakeup event detected.)
            WU_IA: bool = 3,

            /// Wakeup event detection status on X-axis. Default value: 0
            /// (0: wakeup event on X-axis not detected; 1: wakeup event on X-axis detected)
            X_WU: bool = 2,

            /// Wakeup event detection status on Y-axis. Default value: 0
            /// (0: wakeup event on Y-axis not detected; 1: wakeup event on Y-axis detected)
            Y_WU: bool = 1,

            /// Wakeup event detection status on Z-axis. Default value: 0
            /// (0: wakeup event on Z-axis not detected; 1: wakeup event on Z-axis detected)
            Z_WU: bool = 0,

        },

        ///  Status register (r)
        register STATUS_REG {
            const ADDRESS = 0x1E; // 1Eh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Temperature new data available. Default: 0
            /// (0: no set of data is available at temperature sensor output;
            /// 1: a new set of data is available at temperature sensor output)
            TDA: bool = 2,

            /// Gyroscope new data available. Default value: 0
            /// (0: no set of data available at gyroscope output;
            /// 1: a new set of data is available at gyroscope output)
            GDA: bool = 1,

            /// Accelerometer new data available. Default value: 0
            /// (0: no set of data available at accelerometer output;
            /// 1: a new set of data is available at accelerometer output)
            XLDA: bool = 0,

        },

        /// Temperature data output register (r). L and H registers together express a 16-bit word in
        /// two’s complement.
        register OUT_TEMP_L {
            const ADDRESS = 0x20; // 20h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// Temperature sensor output data
            /// The value is expressed as two’s complement sign extended on the MSB.
            Temp: uint = 0..=7,
        },

        /// Temperature data output register (r). L and H registers together express a 16-bit word in
        /// two’s complement.
        register OUT_TEMP_H {
            const ADDRESS = 0x21; // 21h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Temperature sensor output data
            /// The value is expressed as two’s complement sign extended on the MSB.
            Temp: uint = 0..=7,
        },

        /// Temperature Data Output Compound Register (r)
        register OUT_TEMP {
            const ADDRESS = 0x20; // same as OUT_TEMP_L
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output
            // is output only, has no reset.

            /// Temperature sensor output data (16-bit, signed, two’s complement)
            Temp: int = 0..=15,
        },

        /// Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as
        /// a 16-bit word in two’s complement.
        register OUTX_L_G {
            const ADDRESS = 0x22; // 22h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// Pitch axis (X) angular rate value
            AngularRateSensorPitch: uint = 0..=7,
        },

        /// Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as
        /// a 16-bit word in two’s complement.
        register OUTX_H_G {
            const ADDRESS = 0x23; // 23h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Pitch axis (X) angular rate value
            AngularRateSensorPitch: uint = 0..=7,
        },

        /// Angular Rate Sensor Pitch Axis (X) Output Compound Register (r)
        register OUTX_G {
            const ADDRESS = 0x22; // same as OUTX_L_G
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// Pitch axis (X) angular rate value (16-bit, signed, two’s complement)
            AngularRateSensorPitch: int = 0..=15,
        },

        /// Angular rate sensor roll axis (Y) angular rate output register (r). The value is expressed as a
        /// 16-bit word in two’s complement.
        register OUTY_L_G {
            const ADDRESS = 0x24; // 24h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// Roll axis (Y) angular rate value
            AngularRateSensorRoll: uint = 0..=7,
        },

        /// Angular rate sensor roll axis (Y) angular rate output register (r). The value is expressed as a
        /// 16-bit word in two’s complement.
        register OUTY_H_G {
            const ADDRESS = 0x25; // 25h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Roll axis (Y) angular rate value
            AngularRateSensorRoll: uint = 0..=7,
        },

        /// Angular Rate Sensor Roll Axis (Y) Output Compound Register (r)
        register OUTY_G {
            const ADDRESS = 0x24; // same as OUTY_L_G
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// Roll axis (Y) angular rate value (16-bit, signed, two’s complement)
            AngularRateSensorRoll: int = 0..=15,
        },

        /// Angular rate sensor yaw axis (Z) angular rate output register (r). The value is expressed as
        /// a 16-bit word in two’s complement.
        register OUTZ_L_G {
            const ADDRESS = 0x26; // 26h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// Yaw axis (Z) angular rate value
            AngularRateSensorYaw: uint = 0..=7,
        },

        /// Angular rate sensor yaw axis (Z) angular rate output register (r). The value is expressed as
        /// a 16-bit word in two’s complement.
        register OUTZ_H_G {
            const ADDRESS = 0x27; // 27h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Yaw axis (Z) angular rate value
            AngularRateSensorYaw: uint = 0..=7,
        },

        /// Angular Rate Sensor Yaw Axis (Z) Output Compound Register (r)
        register OUTZ_G {
            const ADDRESS = 0x26; // same as OUTZ_L_G
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// Yaw axis (Z) angular rate value (16-bit, signed, two’s complement)
            AngularRateSensorYaw: int = 0..=15,
        },

        /// Linear acceleration sensor X-axis output register (r). The value is expressed as a 16-bit
        /// word in two’s complement.
        register OUTX_L_A {
            const ADDRESS = 0x28; // 28h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// X-axis linear acceleration value
            LinearAccelerationSensorX: uint = 0..=7,
        },

        /// Linear acceleration sensor X-axis output register (r). The value is expressed as a 16-bit
        /// word in two’s complement.
        register OUTX_H_A {
            const ADDRESS = 0x29; // 29h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// X-axis linear acceleration value
            LinearAccelerationSensorX: uint = 0..=7,
        },

        /// Linear Acceleration Sensor X-Axis Output Compound Register (r)
        register OUTX_A {
            const ADDRESS = 0x28; // same as OUTX_L_A
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// X-axis linear acceleration value (16-bit, signed, two’s complement)
            LinearAccelerationSensorX: int = 0..=15,
        },


        /// Linear acceleration sensor Y-axis output register (r). The value is expressed as a 16-bit
        /// word in two’s complement.
        register OUTY_L_A {
            const ADDRESS = 0x2A; // 2Ah
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// Y-axis linear acceleration value
            LinearAccelerationSensorY: uint = 0..=7,
        },

        /// Linear acceleration sensor Y-axis output register (r). The value is expressed as a 16-bit
        /// word in two’s complement.
        register OUTY_H_A {
            const ADDRESS = 0x2B; // 2Bh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Y-axis linear acceleration value
            LinearAccelerationSensorY: uint = 0..=7,
        },

        /// Linear Acceleration Sensor Y-Axis Output Compound Register (r)
        register OUTY_A {
            const ADDRESS = 0x2A; // same as OUTY_L_A
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// Y-axis linear acceleration value (16-bit, signed, two’s complement)
            LinearAccelerationSensorY: int = 0..=15,
        },


        /// Linear acceleration sensor Z-axis output register (r). The value is expressed as a 16-bit
        /// word in two’s complement.
        register OUTZ_L_A {
            const ADDRESS = 0x2C; // 2Ch
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output

            /// Z-axis linear acceleration value
            LinearAccelerationSensorZ: uint = 0..=7,
        },

        /// Linear acceleration sensor Z-axis output register (r). The value is expressed as a 16-bit
        /// word in two’s complement.
        register OUTZ_H_A {
            const ADDRESS = 0x2D; // 2Dh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Z-axis linear acceleration value
            LinearAccelerationSensorZ: uint = 0..=7,
        },

        /// Linear Acceleration Sensor Z-Axis Output Compound Register (r)
        register OUTZ_A {
            const ADDRESS = 0x2C; // same as OUTZ_L_A
            const SIZE_BITS = 16;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// Z-axis linear acceleration value (16-bit, signed, two’s complement)
            LinearAccelerationSensorZ: int = 0..=15,
        },

        /// Gyroscope Output Compound Register (r)
        register OUT_G {
            const ADDRESS = 0x22; // same as OUTX_L_G, first register of the compound
            const SIZE_BITS = 48;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// X-axis angular rate (16-bit, signed, two’s complement)
            AngularRateSensorX: int = 0..=15,

            /// Y-axis angular rate (16-bit, signed, two’s complement)
            AngularRateSensorY: int = 16..=31,

            /// Z-axis angular rate (16-bit, signed, two’s complement)
            AngularRateSensorZ: int = 32..=47,
        },

        /// Accelerometer Output Compound Register (r)
        register OUT_A {
            const ADDRESS = 0x28; // same as OUTX_L_A, first register of the compound
            const SIZE_BITS = 48;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// X-axis linear acceleration (16-bit, signed, two’s complement)
            LinearAccelerationSensorX: int = 0..=15,

            /// Y-axis linear acceleration (16-bit, signed, two’s complement)
            LinearAccelerationSensorY: int = 16..=31,

            /// Z-axis linear acceleration (16-bit, signed, two’s complement)
            LinearAccelerationSensorZ: int = 32..=47,
        },

        /// Combined Gyroscope and Accelerometer Output Compound Register (r)
        register OUT_G_A {
            const ADDRESS = 0x22; // same as OUTX_L_G, first register of the combined compound
            const SIZE_BITS = 96;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            // is output only, has no reset.

            /// X-axis angular rate (16-bit, signed, two’s complement)
            AngularRateSensorX: int = 0..=15,

            /// Y-axis angular rate (16-bit, signed, two’s complement)
            AngularRateSensorY: int = 16..=31,

            /// Z-axis angular rate (16-bit, signed, two’s complement)
            AngularRateSensorZ: int = 32..=47,

            /// X-axis linear acceleration (16-bit, signed, two’s complement)
            LinearAccelerationSensorX: int = 48..=63,

            /// Y-axis linear acceleration (16-bit, signed, two’s complement)
            LinearAccelerationSensorY: int = 64..=79,

            /// Z-axis linear acceleration (16-bit, signed, two’s complement)
            LinearAccelerationSensorZ: int = 80..=95,
        },

        /// FIFO status register 1 (r)
        register FIFO_STATUS1 {
            const ADDRESS = 0x3A; // 3Ah
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Number of unread sensor data (TAG + 6 bytes) stored in FIFO
            /// In conjunction with DIFF_FIFO[9:8] in FIFO_STATUS2 (3Bh).
            DIFF_FIFO: uint = 0..=7,

        },

        /// FIFO status register 2 (r)
        register FIFO_STATUS2 {
            const ADDRESS = 0x3B; // 3Bh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO watermark status. Default value: 0
            /// (0: FIFO filling is lower than WTM;
            /// 1: FIFO filling is equal to or greater than WTM)
            /// Watermark is set through bits WTM[8:0] in FIFO_CTRL2 (08h) and FIFO_CTRL1
            /// (07h).
            FIFO_WTM_IA: bool = 7,

            /// FIFO overrun status. Default value: 0
            /// (0: FIFO is not completely filled; 1: FIFO is completely filled)
            FIFO_OVR_IA: bool = 6,

            /// Smart FIFO full status. Default value: 0
            /// (0: FIFO is not full; 1: FIFO will be full at the next ODR)
            FIFO_FULL_IA: bool = 5,

            /// Counter BDR reaches the CNT_BDR_TH_[10:0] threshold set in
            /// COUNTER_BDR_REG1 (0Bh) and COUNTER_BDR_REG2 (0Ch). Default value: 0
            /// This bit is reset when these registers are read.
            COUNTER_BDR_IA: bool = 4,

            /// Latched FIFO overrun status. Default value: 0
            /// This bit is reset when this register is read.
            FIFO_OVR_LATCHED: bool = 3,

            /// Number of unread sensor data (TAG + 6 bytes) stored in FIFO. Default value: 00
            /// In conjunction with DIFF_FIFO[7:0] in FIFO_STATUS1 (3Ah)
            DIFF_FIFO: uint = 0..=1,

        },

        /// Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit
        /// resolution is 25 μs.
        register TIMESTAMP0 {
            const ADDRESS = 0x40; // 40h
            const SIZE_BITS = 8;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output
            // is output only, has no reset.

            /// Timestamp output registers: 1LSB = 25 μs
            Timestamp: uint =0..=7,
        },

        /// Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit
        /// resolution is 25 μs.
        register TIMESTAMP1 {
            const ADDRESS = 0x41; // 41h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Timestamp output registers: 1LSB = 25 μs
            Timestamp: uint =0..=7,
        },

        /// Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit
        /// resolution is 25 μs.
        register TIMESTAMP2 {
            const ADDRESS = 0x42; // 42h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Timestamp output registers: 1LSB = 25 μs
            Timestamp: uint =0..=7,
        },

        /// Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit
        /// resolution is 25 μs.
        register TIMESTAMP3 {
            const ADDRESS = 0x43; // 43h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Timestamp output registers: 1LSB = 25 μs
            Timestamp: uint =0..=7,
        },

        /// Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit
        /// resolution is 25 μs.
        register TIMESTAMP {
            const ADDRESS = 0x40; // 40h
            const SIZE_BITS = 32;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true; // this is the first register of the compound output
            // is output only, has no reset.

            /// Timestamp output registers: 1LSB = 25 μs
            Timestamp: uint =0..=31,
        },

        /// Accelerometer X-axis user offset correction (r/w). The offset value set in the X_OFS_USR
        /// offset register is internally subtracted from the acceleration value measured on the X-axis.
        register X_OFS_USR {
            const ADDRESS = 0x73; // 73h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Accelerometer X-axis user offset correction expressed in two’s complement,
            /// weight depends on USR_OFF_W in CTRL6_C (15h). The value must be in the
            /// range [-127 127].
            X_OFS_USR: uint  = 0..=7,

        },

        /// Accelerometer Y-axis user offset correction (r/w). The offset value set in the Y_OFS_USR
        /// offset register is internally subtracted from the acceleration value measured on the Y-axis.
        register Y_OFS_USR {
            const ADDRESS = 0x74; // 74h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Accelerometer Y-axis user offset correction expressed in two’s complement,
            /// weight depends on USR_OFF_W in CTRL6_C (15h). The value must be in the
            /// range [-127 127].
            Y_OFS_USR: uint = 0..=7,

        },


        /// Accelerometer Z-axis user offset correction (r/w). The offset value set in the Z_OFS_USR
        /// offset register is internally subtracted from the acceleration value measured on the Z-axis.
        register Z_OFS_USR {
            const ADDRESS = 0x75; // 75h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Accelerometer Z-axis user offset correction expressed in two’s complement,
            /// weight depends on USR_OFF_W in CTRL6_C (15h). The value must be in the
            /// range [-127 127].
            Z_OFS_USR: uint = 0..=7,

        },

        register INTERNAL_FREQ_FINE {
            const ADDRESS = 0x63; // 63h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// Difference in percentage of the effective ODR (and Timestamp Rate) with
            /// respect to the typical. Step: 0.15%. 8-bit format, 2's complement.
            FREQ_FINE: int = 0..=7,

       },

        /*
        /// FIFO tag register (r)
        register FIFO_DATA_OUT_TAG {
            const ADDRESS = 0x78; // 78h
            const SIZE_BITS = 8;
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true; // first register of the compound output

            /// FIFO tag: identifies the sensor (see Table 150)
            TAG_SENSOR: uint as enum TagSensor {
                GyroscopeNC           = 0x01,
                AccelerometerNC       = 0x02,
                Temperature           = 0x03,
                Timestamp             = 0x04,
                CFGChange             = 0x05,
                AccelerometerNC_T_2   = 0x06,
                AccelerometerNC_T_1   = 0x07,
                Accelerometer2xC      = 0x08,
                Accelerometer3xC      = 0x09,
                GyroscopeNC_T_2       = 0x0A,
                GyroscopeNC_T_1       = 0x0B,
                Gyroscope2xC          = 0x0C,
                Gyroscope3xC          = 0x0D,
                SensorHubSlave0       = 0x0E,
                SensorHubSlave1       = 0x0F,
                SensorHubSlave2       = 0x10,
                SensorHubSlave3       = 0x11,
                StepCounter           = 0x12,
                SensorHubNack         = 0x19,
                NotAllowed            = catch_all,
            } = 3..=7,

            /// 2-bit counter which identifies sensor time slot
            TAG_CNT: uint = 1..=2,

            /// Parity check of TAG content
            TAG_PARITY: bool = 0,
        }

         */

        /*
        /// FIFO data output X (r)
        register FIFO_DATA_OUT_X_L {
            const ADDRESS = 0x79; // 79h
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO X-axis output
            FIFODataOutX: uint = 0..=7,
        },

        /// FIFO data output X (r)
        register FIFO_DATA_OUT_X_H {
            const ADDRESS = 0x7A; // 7Ah
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO X-axis output
            FIFODataOutX: uint = 0..=7,
        },

        /// FIFO data output Y (r)
        register FIFO_DATA_OUT_Y_L {
            const ADDRESS = 0x7B; // 7Bh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO Y-axis output
            FIFODataOutY: uint = 0..=7,
        },

        /// FIFO data output Y (r)
        register FIFO_DATA_OUT_Y_H {
            const ADDRESS = 0x7C; // 7Ch
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO Y-axis output
            FIFODataOutY: uint = 0..=7,
        },

        /// FIFO data output Z (r)
        register FIFO_DATA_OUT_Z_L {
            const ADDRESS = 0x7D; // 7Dh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO Z-axis output
            FIFODataOutZ: uint = 0..=7,
        },

        /// FIFO data output Z (r)
        register FIFO_DATA_OUT_Z_H {
            const ADDRESS = 0x7E; // 7Eh
            const SIZE_BITS = 8;
            type Access = RO;
            // is output only, has no reset.

            /// FIFO Z-axis output
            FIFODataOutZ: uint = 0..=7,
        },

        /// FIFO Data Output Compound Register (r)
        /// FIFO Data Output Compound Register (r)
        register FIFO_DATA_OUT {
            const ADDRESS = 0x78;
            const SIZE_BITS = 56; // 8 bits for tag + 48 bits for X,Y,Z
            type Access = RO;
            const ALLOW_ADDRESS_OVERLAP = true;
            const ALLOW_BIT_OVERLAP = true;
            // is output only, has no reset.

            /// FIFO tag: identifies which sensor the data belongs to
            TAG_SENSOR: uint as crate::TagSensor= 3..=7,

            /// 2-bit counter which identifies sensor time slot
            TAG_CNT: uint = 1..=2,

            /// Parity check of TAG content
            TAG_PARITY: uint = 0..=0,

            /// FIFO X-axis output (16-bit, signed)
            FIFODataOutX: int = 8..=23,

            /// FIFO Y-axis output (16-bit, signed)
            FIFODataOutY: int = 24..=39,

            /// FIFO Z-axis output (16-bit, signed)
            FIFODataOutZ: int = 40..=55,

            /* Custom fields for convenience */

            /// FIFO full data output (48-bit, unsigned)
            FIFODataOut: uint = 8..=55,

            /// Full raw data for a fifo entry
            FIFODataRaw: uint = 0..=55,

            Data0: uint = 0..=7,
            Data1: uint = 8..=15,
            Data2: uint = 16..=23,
            Data3: uint = 24..=31,
            Data4: uint = 32..=39,
            Data5: uint = 40..=47,
            Data6: uint = 48..=55,
        }
         */
    }
);

/* Incomplete registers

       register TAP_SRC {
           const ADDRESS = 0x1C; // 1Ch
           const SIZE_BITS = 8;
           type Access = RO;
           // is output only, has no reset.
       },

       register D6D_SRC {
           const ADDRESS = 0x1D; // 1Dh
           const SIZE_BITS = 8;
           type Access = RO;
           // is output only, has no reset.
       },

       register EMB_FUNC_STATUS_MAINPAGE {
           const ADDRESS = 0x35; // 35h
           const SIZE_BITS = 8;
           type Access = RO;
           // is output only, has no reset.
       },

       register FSM_STATUS_A_MAINPAGE {
           const ADDRESS = 0x36; // 36h
           const SIZE_BITS = 8;
           type Access = RO;
           // is output only, has no reset.
       },

       register FSM_STATUS_B_MAINPAGE {
           const ADDRESS = 0x37; // 37h
           const SIZE_BITS = 8;
           type Access = RO;
           // is output only, has no reset.
       },

       register STATUS_MASTER_MAINPAGE {
           const ADDRESS = 0x39; // 39h
           const SIZE_BITS = 8;
           type Access = RO;
           // is output only, has no reset.
       },

               register TAP_CFG0 {
           const ADDRESS = 0x56; // 56h
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register TAP_CFG1 {
           const ADDRESS = 0x57; // 57h
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register TAP_CFG2 {
           const ADDRESS = 0x58; // 58h
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register TAP_THS_6D {
           const ADDRESS = 0x59; // 59h
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register INT_DUR2 {
           const ADDRESS = 0x5A; // 5Ah
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register WAKE_UP_THS {
           const ADDRESS = 0x5B; // 5Bh
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register WAKE_UP_DUR {
           const ADDRESS = 0x5C; // 5Ch
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register FREE_FALL {
           const ADDRESS = 0x5D; // 5Dh
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register MD1_CFG {
           const ADDRESS = 0x5E; // 5Eh
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register MD2_CFG {
           const ADDRESS = 0x5F; // 5Fh
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },

       register I3C_BUS_AVB {
           const ADDRESS = 0x62; // 62h
           const SIZE_BITS = 8;
           type Access = RW;
           const RESET_VALUE = 0x00;
       },
*/
