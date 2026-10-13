//! Formalized <https://www.ti.com/lit/ds/symlink/ina232.pdf>

device_driver::create_device!(
    device_name: Ina232device,
    dsl: {
        config {
            type DefaultRegisterAccess = RO;
            type DefaultFieldAccess = RW;
            type DefaultBufferAccess = RW;
            type DefaultByteOrder = BE;
            type DefaultBitOrder = LSB0;
            type RegisterAddressType = u8;
            type NameWordBoundaries = [
                Underscore, Hyphen, Space, LowerUpper,
            ];
            type DefmtFeature = "defmt-03";
        }

        // Configuration Register – Address 0x00 – R/W – Reset 0x4127
        register CONFIG {
            const ADDRESS = 0x00;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0x4127;

            /// Bit 15: Reset bit (self-clearing)
            RST: bool = 15,
            /// Bits 14-13: Reserved (always 10)
            Reserved: uint = 13..=14,
            /// Bit 12: ADC Range (0 = ±81.92mV, 1 = ±20.48mV)
            ADCRANGE: bool = 12,
            /// Bits 11-9: Averaging mode
            AVG: uint as enum Avg {
                Avg1    = 0b000,
                Avg4    = 0b001,
                Avg16   = 0b010,
                Avg64   = 0b011,
                Avg128  = 0b100,
                Avg256  = 0b101,
                Avg512  = 0b110,
                Avg1024 = 0b111
            } = 9..=11,
            /// Bits 8-6: Bus voltage conversion time
            VBUSCT: uint as enum VbusCt {
                Ct140us  = 0b000,
                Ct204us  = 0b001,
                Ct332us  = 0b010,
                Ct588us  = 0b011,
                Ct1100us = 0b100,
                Ct2116us = 0b101,
                Ct4156us = 0b110,
                Ct8244us = 0b111
            } = 6..=8,
            /// Bits 5-3: Shunt voltage conversion time
            VSHCT: uint as enum VshCt {
                Ct140us  = 0b000,
                Ct204us  = 0b001,
                Ct332us  = 0b010,
                Ct588us  = 0b011,
                Ct1100us = 0b100,
                Ct2116us = 0b101,
                Ct4156us = 0b110,
                Ct8244us = 0b111
            } = 3..=5,
            /// Bits 2-0: Operating mode
            MODE: uint as enum Mode {
                Shutdown         = 0b000,
                ShuntTriggered   = 0b001,
                BusTriggered     = 0b010,
                ShuntBusTriggered = 0b011,
                ShutdownAlt      = 0b100,
                ShuntContinuous  = 0b101,
                BusContinuous    = 0b110,
                ShuntBusContinuous = 0b111
            } = 0..=2,
        },

        // Shunt Voltage Register – Address 0x01 – RO – Reset 0x0000
        register SHUNT_VOLTAGE {
            const ADDRESS = 0x01;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0x0000;

            /// Differential voltage across shunt (two's complement)
            VSHUNT: int = 0..=15,
        },

        // Bus Voltage Register – Address 0x02 – RO – Reset 0x0000
        register BUS_VOLTAGE {
            const ADDRESS = 0x02;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0x0000;

            /// Bus voltage (lower 15 bits, bit 15 reserved = 0)
            VBUS: uint = 0..=14,
        },

        // Power Register – Address 0x03 – RO – Reset 0x0000
        register POWER {
            const ADDRESS = 0x03;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0x0000;

            /// Calculated power (unsigned)
            POWER: uint = 0..=15,
        },

        // Current Register – Address 0x04 – RO – Reset 0x0000
        register CURRENT {
            const ADDRESS = 0x04;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0x0000;

            /// Calculated current in amperes (two's complement)
            CURRENT: int = 0..=15,
        },

        // Calibration Register – Address 0x05 – R/W – Reset 0x0000
        register CALIBRATION {
            const ADDRESS = 0x05;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0x0000;

            /// Shunt calibration value (lower 15 bits, bit 15 reserved)
            SHUNT_CAL: uint = 0..=14,
        },

        // Mask/Enable Register – Address 0x06 – R/W – Reset 0x0000
        register MASK_ENABLE {
            const ADDRESS = 0x06;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0x0000;
            // Not fully expanded - only what we might need
            _RESERVED: uint = 0..=15,
        },

        // Alert Limit Register – Address 0x07 – R/W – Reset 0x0000
        register ALERT_LIMIT {
            const ADDRESS = 0x07;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0x0000;

            LIMIT: uint = 0..=15,
        },

        // Manufacturer ID Register – Address 0x3E – RO – Reset 0x5449
        register MANUFACTURER_ID {
            const ADDRESS = 0x3E;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0x5449;

            /// Reads back "TI" in ASCII
            MANUFACTURER_ID: uint = 0..=15,
        },
    }
);
