device_driver::create_device!(
    device_name: Ltc2945device,
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

        ////////////////////////////////////////////////////////////////////////////////
        //  CONTROL Register (A) – Address 0x00 – R/W – Reset 0x05
        //  Controls ADC operation mode and test mode
        ////////////////////////////////////////////////////////////////////////////////
        register CONTROL_A {
            const ADDRESS = 0x00; // 00h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x05;

            /// Bit 7: Enables ADC Snapshot Mode. Default: 0
            /// (0: Continuous mode; 1: Snapshot mode)
            AdcSnapshotEnable: bool = 7,

            /// Bits 6..5: ADC channel for snapshot mode
            /// 00: ∆SENSE (Default)
            /// 01: VIN
            /// 10: ADIN
            /// 11: Reserved
            AdcChannel: uint as enum AdcChannel {
                DeltaSense = 0b00,
                Vin        = 0b01,
                Adin       = 0b10,
                Reserved   = 0b11
            } = 5..=6,

            /// Bit 4: Test Mode Enable (A4)
            /// Set this to 1 to allow writing to ADC & MIN/MAX ADC registers.
            TestModeEnable: bool = 4,

            /// Bit 3: ADC Busy (read-only status)
            /// (0: Not busy; 1: Converting)
            AdcBusy: bool = 3,

            /// Bit 2: Monitor select. Default: 1
            /// (0: Monitor VDD; 1: Monitor SENSE+)
            VinMonitor: bool = 2,

            /// Bit 1: Shutdown enable. Default: 0
            /// (0: Normal; 1: Shutdown)
            ShutdownEnable: bool = 1,

            /// Bit 0: Multiplier Select for power calc
            /// (0: Use ADIN; 1: Use SENSE+/VDD)
            MultiplierSelect: bool = 0,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  ALERT Register (B) – Address 0x01 – R/W – Reset 0x00
        //  Selects which fault conditions generate alerts
        ////////////////////////////////////////////////////////////////////////////////
        register ALERT_B {
            const ADDRESS = 0x01; // 01h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Bit 7: Alert if POWER > Max POWER threshold
            MaxPowerAlertEnable: bool = 7,
            /// Bit 6: Alert if POWER < Min POWER threshold
            MinPowerAlertEnable: bool = 6,
            /// Bit 5: Alert if ∆SENSE > Max ∆SENSE threshold
            MaxDeltaSenseAlertEnable: bool = 5,
            /// Bit 4: Alert if ∆SENSE < Min ∆SENSE threshold
            MinDeltaSenseAlertEnable: bool = 4,
            /// Bit 3: Alert if VIN > Max VIN threshold
            MaxVinAlertEnable: bool = 3,
            /// Bit 2: Alert if VIN < Min VIN threshold
            MinVinAlertEnable: bool = 2,
            /// Bit 1: Alert if ADIN > Max ADIN threshold
            MaxAdinAlertEnable: bool = 1,
            /// Bit 0: Alert if ADIN < Min ADIN threshold
            MinAdinAlertEnable: bool = 0,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  STATUS Register (C) – Address 0x02 – R – Reset 0x00
        //  System status information (live flags)
        ////////////////////////////////////////////////////////////////////////////////
        register STATUS_C {
            const ADDRESS = 0x02; // 02h
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0x00;

            /// Bit 7: POWER Overvalue present
            PowerOvervalue: bool = 7,
            /// Bit 6: POWER Undervalue present
            PowerUndervalue: bool = 6,
            /// Bit 5: ∆SENSE Overvalue present
            DeltaSenseOvervalue: bool = 5,
            /// Bit 4: ∆SENSE Undervalue present
            DeltaSenseUndervalue: bool = 4,
            /// Bit 3: VIN Overvalue present
            VinOvervalue: bool = 3,
            /// Bit 2: VIN Undervalue present
            VinUndervalue: bool = 2,
            /// Bit 1: ADIN Overvalue present
            AdinOvervalue: bool = 1,
            /// Bit 0: ADIN Undervalue present
            AdinUndervalue: bool = 0,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  FAULT Register (D) – Address 0x03 – R/W – Reset 0x00
        //  Fault log (latched bits)
        ////////////////////////////////////////////////////////////////////////////////
        register FAULT_D {
            const ADDRESS = 0x03; // 03h
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            /// Bit 7: POWER overvalue fault occurred
            PowerOvervalueFault: bool = 7,
            /// Bit 6: POWER undervalue fault occurred
            PowerUndervalueFault: bool = 6,
            /// Bit 5: ∆SENSE overvalue fault occurred
            DeltaSenseOvervalueFault: bool = 5,
            /// Bit 4: ∆SENSE undervalue fault occurred
            DeltaSenseUndervalueFault: bool = 4,
            /// Bit 3: VIN overvalue fault occurred
            VinOvervalueFault: bool = 3,
            /// Bit 2: VIN undervalue fault occurred
            VinUndervalueFault: bool = 2,
            /// Bit 1: ADIN overvalue fault occurred
            AdinOvervalueFault: bool = 1,
            /// Bit 0: ADIN undervalue fault occurred
            AdinUndervalueFault: bool = 0,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  FAULT_CoR Register (E) – Address 0x04 – R – Reset 0x00
        //  Same data as FAULT(D). Reading clears FAULT bits
        ////////////////////////////////////////////////////////////////////////////////
        register FAULT_CoR_E {
            const ADDRESS = 0x04; // 04h
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0x00;

            /// Bit layout same as FAULT(D). Reading this register clears FAULT(D).
            PowerOvervalueFault: bool = 7,
            PowerUndervalueFault: bool = 6,
            DeltaSenseOvervalueFault: bool = 5,
            DeltaSenseUndervalueFault: bool = 4,
            VinOvervalueFault: bool = 3,
            VinUndervalueFault: bool = 2,
            AdinOvervalueFault: bool = 1,
            AdinUndervalueFault: bool = 0,
        },

        ////////////////////////////////////////////////////////////////////////////
        //  POWER – Address 0x05 (24‑bit) – R/W** – Reset: 0x000000
        ////////////////////////////////////////////////////////////////////////////
        register POWER {
            const ADDRESS   = 0x05;
            const SIZE_BITS = 24;
            type  Access    = RW;
            const RESET_VALUE = 0x000000;
            Power: uint = 0..=23,
        },


        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_POWER MSB2 – Address 0x08 – R/W** – Reset: 0x00
        //  Max recorded POWER (24 bits). Write requires TestModeEnable=1.
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_POWER_MSB2 {
            const ADDRESS = 0x08;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MAX_POWER_MSB2: uint = 0..=7,
        },

        register MAX_POWER_MSB1 {
            const ADDRESS = 0x09;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MAX_POWER_MSB1: uint = 0..=7,
        },

        register MAX_POWER_LSB {
            const ADDRESS = 0x0A;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MAX_POWER_LSB: uint = 0..=7,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MIN_POWER MSB2 – Address 0x0B – R/W** – Reset: 0xFF
        //  Min recorded POWER. Write requires TestModeEnable=1.
        ////////////////////////////////////////////////////////////////////////////////
        register MIN_POWER_MSB2 {
            const ADDRESS = 0x0B;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MIN_POWER_MSB2: uint = 0..=7,
        },

        register MIN_POWER_MSB1 {
            const ADDRESS = 0x0C;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MIN_POWER_MSB1: uint = 0..=7,
        },

        register MIN_POWER_LSB {
            const ADDRESS = 0x0D;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MIN_POWER_LSB: uint = 0..=7,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_POWER_THRESHOLD MSB2 – Address 0x0E – R/W – Reset: 0xFF
        //  Threshold is always R/W in normal mode (no test mode required)
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_POWER_THRESHOLD_MSB2 {
            const ADDRESS = 0x0E;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MAX_POWER_THRESHOLD_MSB2: uint = 0..=7,
        },

        register MAX_POWER_THRESHOLD_MSB1 {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MAX_POWER_THRESHOLD_MSB1: uint = 0..=7,
        },

        register MAX_POWER_THRESHOLD_LSB {
            const ADDRESS = 0x10;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MAX_POWER_THRESHOLD_LSB: uint = 0..=7,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MIN_POWER_THRESHOLD MSB2 – Address 0x11 – R/W – Reset: 0x00
        //  Also R/W in normal mode
        ////////////////////////////////////////////////////////////////////////////////
        register MIN_POWER_THRESHOLD_MSB2 {
            const ADDRESS = 0x11;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MIN_POWER_THRESHOLD_MSB2: uint = 0..=7,
        },

        register MIN_POWER_THRESHOLD_MSB1 {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MIN_POWER_THRESHOLD_MSB1: uint = 0..=7,
        },

        register MIN_POWER_THRESHOLD_LSB {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MIN_POWER_THRESHOLD_LSB: uint = 0..=7,
        },

        ////////////////////////////////////////////////////////////////////////////
        //  ∆SENSE – Address 0x14 (16‑bit) – R/W** – Reset: 0x0000
        ////////////////////////////////////////////////////////////////////////////
        register DELTA_SENSE {
            const ADDRESS   = 0x14;
            const SIZE_BITS = 16;
            type  Access    = RW;
            const RESET_VALUE = 0x0000;
            DeltaSense: uint = 0..=15,
        },


        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_DELTA_SENSE MSB – Address 0x16 – R/W** – Reset: 0x00
        //  Max recorded ∆SENSE. Write requires TestModeEnable=1.
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_DELTA_SENSE_MSB {
            const ADDRESS = 0x16;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MAX_DELTA_SENSE_MSB: uint = 0..=7,
        },

        register MAX_DELTA_SENSE_LSB {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MIN_DELTA_SENSE MSB – Address 0x18 – R/W** – Reset: 0xFF
        ////////////////////////////////////////////////////////////////////////////////
        register MIN_DELTA_SENSE_MSB {
            const ADDRESS = 0x18;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MIN_DELTA_SENSE_MSB: uint = 0..=7,
        },

        register MIN_DELTA_SENSE_LSB {
            const ADDRESS = 0x19;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xF0;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_DELTA_SENSE_THRESHOLD MSB – Address 0x1A – R/W – Reset: 0xFF
        //  Threshold registers: fully R/W
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_DELTA_SENSE_THRESHOLD_MSB {
            const ADDRESS = 0x1A;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MAX_DELTA_SENSE_THRESHOLD_MSB: uint = 0..=7,
        },

        register MAX_DELTA_SENSE_THRESHOLD_LSB {
            const ADDRESS = 0x1B;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xF0;

            DataLow: uint = 4..=7,

        },

        register MIN_DELTA_SENSE_THRESHOLD_MSB {
            const ADDRESS = 0x1C;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MIN_DELTA_SENSE_THRESHOLD_MSB: uint = 0..=7,
        },

        register MIN_DELTA_SENSE_THRESHOLD_LSB {
            const ADDRESS = 0x1D;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  VIN MSB – Address 0x1E – R/W** – Reset: 0x00
        //  12-bit VIN result. Write requires TestModeEnable=1.
        //  datasheet says "XXh" => undefined, using 0x00
        ////////////////////////////////////////////////////////////////////////////////
        register VIN_MSB {
            const ADDRESS = 0x1E;
            const SIZE_BITS = 8;
            type Access = RW;
            const ALLOW_ADDRESS_OVERLAP = true;
            // "XXh" => undefined, using 0x00
            VIN_MSB: uint = 0..=7,
        },

        register VIN_LSB {
            const ADDRESS = 0x1F;
            const SIZE_BITS = 8;
            type Access = RW;
            // "X0h" => undefined, using 0x00
            DataLow: uint = 4..=7,
        },

        register VIN {
            const ADDRESS = 0x1E;
            const SIZE_BITS = 16;
            type Access = RW;
            const ALLOW_ADDRESS_OVERLAP = true;
            Vin: uint = 0..=15,
        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_VIN MSB – Address 0x20 – R/W** – Reset: 0x00
        //  Max recorded VIN
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_VIN_MSB {
            const ADDRESS = 0x20;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MAX_VIN_MSB: uint = 0..=7,
        },

        register MAX_VIN_LSB {
            const ADDRESS = 0x21;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MIN_VIN MSB – Address 0x22 – R/W** – Reset: 0xFF
        ////////////////////////////////////////////////////////////////////////////////
        register MIN_VIN_MSB {
            const ADDRESS = 0x22;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MIN_VIN_MSB: uint = 0..=7,
        },

        register MIN_VIN_LSB {
            const ADDRESS = 0x23;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xF0;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_VIN_THRESHOLD MSB – Address 0x24 – R/W – Reset: 0xFF
        //  Threshold registers = normal R/W
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_VIN_THRESHOLD_MSB {
            const ADDRESS = 0x24;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MAX_VIN_THRESHOLD_MSB: uint = 0..=7,
        },

        register MAX_VIN_THRESHOLD_LSB {
            const ADDRESS = 0x25;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xF0;

            DataLow: uint = 4..=7,

        },

        register MIN_VIN_THRESHOLD_MSB {
            const ADDRESS = 0x26;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MIN_VIN_THRESHOLD_MSB: uint = 0..=7,
        },

        register MIN_VIN_THRESHOLD_LSB {
            const ADDRESS = 0x27;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  ADIN MSB – Address 0x28 – R/W** – Reset: 0x00
        //  12-bit ADIN result. Write requires TestModeEnable=1.
        //  datasheet says "XXh" => undefined
        ////////////////////////////////////////////////////////////////////////////////
        register ADIN_MSB {
            const ADDRESS = 0x28;
            const SIZE_BITS = 8;
            type Access = RW;
            // "XXh" => undefined, using 0x00
            const RESET_VALUE = 0x00;

            ADIN_MSB: uint = 0..=7,
        },

        register ADIN_LSB {
            const ADDRESS = 0x29;
            const SIZE_BITS = 8;
            type Access = RW;
            // "X0h" => undefined, using 0x00
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_ADIN MSB – Address 0x2A – R/W** – Reset: 0x00
        //  Max recorded ADIN
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_ADIN_MSB {
            const ADDRESS = 0x2A;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MAX_ADIN_MSB: uint = 0..=7,
        },

        register MAX_ADIN_LSB {
            const ADDRESS = 0x2B;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MIN_ADIN MSB – Address 0x2C – R/W** – Reset: 0xFF
        ////////////////////////////////////////////////////////////////////////////////
        register MIN_ADIN_MSB {
            const ADDRESS = 0x2C;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MIN_ADIN_MSB: uint = 0..=7,
        },

        register MIN_ADIN_LSB {
            const ADDRESS = 0x2D;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xF0;

            DataLow: uint = 4..=7,

        },

        ////////////////////////////////////////////////////////////////////////////////
        //  MAX_ADIN_THRESHOLD MSB – Address 0x2E – R/W – Reset: 0xFF
        //  Threshold registers = normal R/W
        ////////////////////////////////////////////////////////////////////////////////
        register MAX_ADIN_THRESHOLD_MSB {
            const ADDRESS = 0x2E;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xFF;

            MAX_ADIN_THRESHOLD_MSB: uint = 0..=7,
        },

        register MAX_ADIN_THRESHOLD_LSB {
            const ADDRESS = 0x2F;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0xF0;

            DataLow: uint = 4..=7,

        },

        register MIN_ADIN_THRESHOLD_MSB {
            const ADDRESS = 0x30;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            MIN_ADIN_THRESHOLD_MSB: uint = 0..=7,
        },

        register MIN_ADIN_THRESHOLD_LSB {
            const ADDRESS = 0x31;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0x00;

            DataLow: uint = 4..=7,

        }
    }
);
