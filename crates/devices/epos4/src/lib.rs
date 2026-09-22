// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![allow(dead_code)]
#![allow(clippy::bool_comparison)]

use core::fmt::Debug;
use embedded_io_async::{Read, ReadExactError, Write};

/// The primary error type for the EPOS4 driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Epos4Error<E: Debug> {
    /// An error occurred in the underlying communication interface.
    Interface(E),
    /// A protocol framing error occurred (e.g., bad sync bytes, CRC mismatch).
    Framing,
    /// The EPOS4 device is in a FAULT state and cannot perform the requested operation.
    DeviceInFaultState,
    /// The EPOS4 device reported a specific error in its response.
    Device(DeviceError),
    /// A timeout occurred while waiting for a response from the EPOS4 device.
    Timeout,
}

/// Represents error codes returned by the EPOS4 controller in a response frame.
///
/// These codes correspond to the SDO Abort Codes in the
/// "EPOS Command Library" documentation, Table 8-32, page 147.
#[repr(u32)]
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceError {
    NoError = 0x0000_0000,
    ToggleError = 0x0503_0000,
    SdoTimeout = 0x0504_0000,
    ClientServerSpecifierError = 0x0504_0001,
    InvalidBlockSize = 0x0504_0002,
    InvalidSequence = 0x0504_0003,
    SdoCrcError = 0x0504_0004,
    OutOfMemoryError = 0x0504_0005,
    AccessError = 0x0601_0000,
    WriteOnlyError = 0x0601_0001,
    ReadOnlyError = 0x0601_0002,
    ObjectNotExistError = 0x0602_0000,
    PdoMappingError = 0x0604_0041,
    PdoLengthError = 0x0604_0042,
    GeneralParameterError = 0x0604_0043,
    GeneralInternalIncompatibilityError = 0x0604_0047,
    HardwareError = 0x0606_0000,
    ServiceParameterError = 0x0607_0010,
    ServiceParameterTooHigh = 0x0607_0012,
    ServiceParameterTooLow = 0x0607_0013,
    SubindexError = 0x0609_0011,
    ValueRangeError = 0x0609_0030,
    ValueTooHigh = 0x0609_0031,
    ValueTooLow = 0x0609_0032,
    MaxLessThanMinError = 0x0609_0036,
    GeneralError = 0x0800_0000,
    TransferOrStoreError = 0x0800_0020,
    LocalControlError = 0x0800_0021,
    WrongDeviceStateError = 0x0800_0022,
    CanIdError = 0x0F00_FFB9,
    PasswordError = 0x0F00_FFBE,
    IllegalCommandError = 0x0F00_FFBF,
    WrongNmtStateError = 0x0F00_FFC0,
    Unknown(u32),
}

impl From<[u16; 2]> for DeviceError {
    /// Converts the 32-bit error code from a response frame into a `DeviceError`.
    fn from(value: [u16; 2]) -> Self {
        let value = (value[0] as u32) | ((value[1] as u32) << 16);
        match value {
            0x0000_0000 => DeviceError::NoError,
            0x0503_0000 => DeviceError::ToggleError,
            0x0504_0000 => DeviceError::SdoTimeout,
            0x0504_0001 => DeviceError::ClientServerSpecifierError,
            0x0504_0002 => DeviceError::InvalidBlockSize,
            0x0504_0003 => DeviceError::InvalidSequence,
            0x0504_0004 => DeviceError::SdoCrcError,
            0x0504_0005 => DeviceError::OutOfMemoryError,
            0x0601_0000 => DeviceError::AccessError,
            0x0601_0001 => DeviceError::WriteOnlyError,
            0x0601_0002 => DeviceError::ReadOnlyError,
            0x0602_0000 => DeviceError::ObjectNotExistError,
            0x0604_0041 => DeviceError::PdoMappingError,
            0x0604_0042 => DeviceError::PdoLengthError,
            0x0604_0043 => DeviceError::GeneralParameterError,
            0x0604_0047 => DeviceError::GeneralInternalIncompatibilityError,
            0x0606_0000 => DeviceError::HardwareError,
            0x0607_0010 => DeviceError::ServiceParameterError,
            0x0607_0012 => DeviceError::ServiceParameterTooHigh,
            0x0607_0013 => DeviceError::ServiceParameterTooLow,
            0x0609_0011 => DeviceError::SubindexError,
            0x0609_0030 => DeviceError::ValueRangeError,
            0x0609_0031 => DeviceError::ValueTooHigh,
            0x0609_0032 => DeviceError::ValueTooLow,
            0x0609_0036 => DeviceError::MaxLessThanMinError,
            0x0800_0000 => DeviceError::GeneralError,
            0x0800_0020 => DeviceError::TransferOrStoreError,
            0x0800_0021 => DeviceError::LocalControlError,
            0x0800_0022 => DeviceError::WrongDeviceStateError,
            0x0F00_FFB9 => DeviceError::CanIdError,
            0x0F00_FFBE => DeviceError::PasswordError,
            0x0F00_FFBF => DeviceError::IllegalCommandError,
            0x0F00_FFC0 => DeviceError::WrongNmtStateError,
            other => DeviceError::Unknown(other),
        }
    }
}

//================================================================================
// Data Structures and Enums
//================================================================================

/// Represents the CiA 402 state machine status of the EPOS4 controller.
/// Based on "EPOS4 Firmware Specification", Table 2-5, page 14.
#[repr(u8)]
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EposStatus {
    NotReadyToSwitchOn = 0,
    SwitchOnDisabled = 0b100_0000,
    ReadyToSwitchOn = 0b010_0001,
    SwitchedOn = 0b010_0011,
    OperationEnabled = 0b010_0111,
    QuickStopActive = 0b000_0111,
    FaultReactionActive = 0b000_1111,
    Fault = 0b000_1000,
    Unknown,
}

impl From<u16> for EposStatus {
    /// Converts a 16-bit `StatusWord` into an `EposStatus`.
    fn from(status_word: u16) -> Self {
        // Bits 6, 5, 4, 3, 2, 1, 0 define the state.
        const MASK: u16 = 0b0110_1111;
        match status_word & MASK {
            0b0000_0000 => EposStatus::NotReadyToSwitchOn,
            0b0100_0000 => EposStatus::SwitchOnDisabled,
            0b0010_0001 => EposStatus::ReadyToSwitchOn,
            0b0010_0011 => EposStatus::SwitchedOn,
            0b0010_0111 => EposStatus::OperationEnabled,
            0b0000_0111 => EposStatus::QuickStopActive,
            0b0000_1111 => EposStatus::FaultReactionActive,
            0b0000_1000 => EposStatus::Fault,
            _ => EposStatus::Unknown,
        }
    }
}

/// Defines the motor type.
/// Based on "EPOS Command Library", Table 4-6, page 40.
#[repr(u16)]
#[derive(Debug, Clone, Copy)]
pub enum MotorType {
    BrushedDcMotor = 1,
    EcMotorSinusCommutated = 10,
    EcMotorBlockCommutated = 11,
}

/// Defines the homing method to be used by the `find_home` command.
/// Based on "EPOS Command Library", Table 5-22, page 91.
#[repr(i8)]
#[derive(Debug, Clone, Copy)]
pub enum HomingMethod {
    CurrentThresholdNegativeSpeed = -4,
    CurrentThresholdPositiveSpeed = -3,
    CurrentThresholdNegativeSpeedAndIndex = -2,
    CurrentThresholdPositiveSpeedAndIndex = -1,
    NegativeLimitSwitchAndIndex = 1,
    PositiveLimitSwitchAndIndex = 2,
    HomeSwitchPositiveSpeedAndIndex = 7,
    HomeSwitchNegativeSpeedAndIndex = 11,
    NegativeLimitSwitch = 17,
    PositiveLimitSwitch = 18,
    HomeSwitchPositiveSpeed = 23,
    HomeSwitchNegativeSpeed = 27,
    IndexNegativeSpeed = 33,
    IndexPositiveSpeed = 34,
    CurrentPosition = 37,
}

/// Represents the status of the homing procedure.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HomingStatus {
    pub attained: bool,
    pub error: bool,
}

//================================================================================
// Main Driver Struct
//================================================================================

/// Driver for the Maxon EPOS4 motor controller via an asynchronous serial interface (Maxon Serial V2).
///
/// **NOTE:** This driver implementation is not exhaustive. It covers common operating modes
/// like PPM, PVM, and HM. It also provides a low-level interface (`read_object`, `write_object`)
/// that is currently limited to 32-bit data types, which covers many, but not all, objects.
pub struct Epos4<R, W>
where
    R: Read,
    W: Write,
{
    node_id: u8,
    rx: R,
    tx: W,
}

impl<R, W, E> Epos4<R, W>
where
    R: Read<Error = E>,
    W: Write<Error = E>,
    E: Debug,
{
    // Protocol constants
    const DATA_LINK_ESCAPE: u8 = 0x90;
    const START_OF_TEXT: u8 = 0x02;

    // --- CiA 402 Object Dictionary Indices ---
    const CONTROL_WORD: u16 = 0x6040;
    const STATUS_WORD: u16 = 0x6041;
    const MODES_OF_OPERATION: u16 = 0x6060;
    const POSITION_ACTUAL_VALUE: u16 = 0x6064;
    const VELOCITY_ACTUAL_VALUE: u16 = 0x606C;
    const TARGET_POSITION: u16 = 0x607A;
    const PROFILE_VELOCITY: u16 = 0x6081;
    const PROFILE_ACCELERATION: u16 = 0x6083;
    const PROFILE_DECELERATION: u16 = 0x6084;
    const HOMING_METHOD: u16 = 0x6098;
    const HOMING_SPEEDS: u16 = 0x6099;
    const HOMING_ACCELERATION: u16 = 0x609A;
    const TARGET_VELOCITY: u16 = 0x60FF;

    // --- Manufacturer-specific Object Dictionary Indices ---
    const MOTOR_DATA: u16 = 0x3001;
    const HOME_OFFSET_MOVE_DISTANCE: u16 = 0x30B1;
    const CURRENT_ACTUAL_VALUES: u16 = 0x30D1;
    const PROGRAM_CONTROL: u16 = 0x1F51;
    const MOTOR_TYPE: u16 = 0x6402;

    /// Creates a new EPOS4 driver instance.
    ///
    /// # Arguments
    /// * `rx` - An asynchronous readable serial interface.
    /// * `tx` - An asynchronous writable serial interface.
    /// * `node_id` - The Node-ID of the EPOS4 controller (1-127).
    pub fn new(rx: R, tx: W, node_id: u8) -> Self {
        Self { node_id, rx, tx }
    }

    /// Sets the current position counter to zero (no movement).
    /// Writes to object 0x2062 ("Set Actual Position Command").
    /// This immediately resets the internal position reference.
    pub async fn set_current_position_zero(&mut self) -> Result<(), Epos4Error<E>> {
        let cur = self.get_current_position().await?;
        self.write_object(0x30B0, 0x0, cur as u32).await?;
        Ok(())
    }

    // --- Configuration Functions ---

    /// Sets the motor type. This should be one of the first configuration steps.
    /// References object `0x6402`. See "EPOS4 Firmware Specification", page 227.
    pub async fn set_motor_type(&mut self, motor_type: MotorType) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::MOTOR_TYPE, 0x00, motor_type as u32)
            .await
    }

    /// Configures the core electrical parameters for an EC (brushless) motor.
    /// This is crucial for correct and safe operation.
    /// This function writes to sub-indices of the "Motor data" object (`0x3001`).
    ///
    /// # Arguments
    /// * `nominal_current`: Max continuous current in mA (Object 0x3001/01).
    /// * `max_output_current`: Max peak current in mA (Object 0x3001/02).
    /// * `thermal_time_constant`: Winding thermal time constant in 0.1s (Object 0x3001/04).
    /// * `pole_pairs`: Number of pole pairs for the motor (Object 0x3001/05).
    ///
    /// **Reference:** "EPOS4 Firmware Specification", pages 145-147.
    pub async fn set_ec_motor_params(
        &mut self,
        nominal_current: u32,
        max_output_current: u32,
        thermal_time_constant: u16,
        pole_pairs: u8,
    ) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::MOTOR_DATA, 0x01, nominal_current)
            .await?;
        self.write_object(Self::MOTOR_DATA, 0x02, max_output_current)
            .await?;
        self.write_object(Self::MOTOR_DATA, 0x04, thermal_time_constant as u32)
            .await?;
        self.write_object(Self::MOTOR_DATA, 0x05, pole_pairs as u32)
            .await?;
        Ok(())
    }

    // --- State Machine Control ---

    /// Transitions the device to the `Operation Enabled` state, allowing motor movement.
    /// This follows the standard CiA 402 state transition path: Shutdown -> Switch On -> Enable.
    pub async fn enable(&mut self) -> Result<(), Epos4Error<E>> {
        self.shutdown().await?; // Ensures we are in `Ready to Switch On`
        self.write_object(Self::CONTROL_WORD, 0x00, 0x0007).await?; // -> Switched On
        self.write_object(Self::CONTROL_WORD, 0x00, 0x000F).await?; // -> Operation Enabled
        Ok(())
    }

    /// Triggers a Quick Stop, halting the motor using the configured Quick Stop deceleration.
    pub async fn quick_stop(&mut self) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::CONTROL_WORD, 0x00, 0x000B).await?;
        Ok(())
    }

    /// Transitions the device to the `Ready To Switch On` state. The motor drive is disabled.
    pub async fn shutdown(&mut self) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::CONTROL_WORD, 0x00, 0x0006).await?;
        Ok(())
    }

    /// Resets the EPOS4 node via a software command.
    /// References "Program control" object `0x1F51`, value `0x02`.
    /// See "EPOS4 Firmware Specification", page 119.
    pub async fn reset_node(&mut self) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::PROGRAM_CONTROL, 0x01, 0x02).await
    }

    /// Clears any fault conditions. If the device is in a `Fault` state, this transitions it
    /// to `Switch On Disabled`.
    pub async fn clear_fault(&mut self) -> Result<(), Epos4Error<E>> {
        // To clear a fault, bit 7 of the ControlWord must be transitioned from 0 to 1.
        // After the device acknowledges the reset, the host must set bit 7 back to 0.
        // See EPOS4 Firmware Specification, Table 2-7, page 15.
        if self.get_status().await? != EposStatus::Fault {
            // Not in fault, nothing to do.
            return Ok(());
        }

        // Trigger the fault reset.
        self.write_object(Self::CONTROL_WORD, 0x00, 0x0080).await?;

        // Allow new state machine commands (optional, as next command will overwrite).
        self.write_object(Self::CONTROL_WORD, 0x00, 0x0000).await?;

        if self.get_status().await? == EposStatus::Fault {
            // The fault is persistent and could not be cleared
            return Err(Epos4Error::DeviceInFaultState);
        }
        Ok(())
    }

    // --- Profile Position Mode (PPM) ---

    /// Sets the operational mode to Profile Position Mode (PPM). Mode `1`.
    /// See "EPOS Command Library", page 81.
    pub async fn set_ppm_mode(&mut self) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::MODES_OF_OPERATION, 0x00, 1).await
    }

    pub async fn set_homing_mode(&mut self) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::MODES_OF_OPERATION, 0x00, 6).await
    }

    /// Sets the velocity, acceleration, and deceleration for PPM movements.
    /// See objects `0x6081`, `0x6083`, `0x6084`.
    pub async fn set_ppm_profile(
        &mut self,
        velocity: u32,
        acceleration: u32,
        deceleration: u32,
    ) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::PROFILE_VELOCITY, 0x00, velocity)
            .await?;
        self.write_object(Self::PROFILE_ACCELERATION, 0x00, acceleration)
            .await?;
        self.write_object(Self::PROFILE_DECELERATION, 0x00, deceleration)
            .await?;
        Ok(())
    }

    /// Commands the motor to move to a new absolute position in PPM.
    ///
    /// # Arguments
    /// * `target_pos` - The absolute target position (Object 0x607A).
    /// * `immediate` - If `true`, the new move overrides any ongoing movement (ControlWord bit 5).
    ///
    /// **Reference:** "EPOS4 Firmware Specification", page 23.
    pub async fn move_to_position(
        &mut self,
        target_pos: i32,
        immediate: bool,
    ) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::TARGET_POSITION, 0x00, target_pos as u32)
            .await?;
        let enable_op_word = 0x000F; // Base word for Operation Enabled
        // First, ensure we are in a state that can accept the new setpoint
        self.write_object(Self::CONTROL_WORD, 0x00, enable_op_word)
            .await?;
        // Then, set the "new setpoint" bit (4) and optionally "immediate" bit (5)
        let start_move_word = if immediate {
            enable_op_word | (1 << 4) | (1 << 5)
        } else {
            enable_op_word | (1 << 4)
        };
        self.write_object(Self::CONTROL_WORD, 0x00, start_move_word)
            .await?;
        // Finally, clear the command bits to allow the move to execute
        self.write_object(Self::CONTROL_WORD, 0x00, enable_op_word)
            .await?;
        Ok(())
    }

    // --- Profile Velocity Mode (PVM) ---

    /// Sets the operational mode to Profile Velocity Mode (PVM). Mode `3`.
    /// See "EPOS Command Library", page 85.
    pub async fn set_pvm_mode(&mut self) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::MODES_OF_OPERATION, 0x00, 3).await
    }

    /// Commands the motor to start moving at a target velocity (Object `0x60FF`).
    /// The motor will accelerate using the currently set profile acceleration.
    pub async fn move_with_velocity(&mut self, target_velocity: i32) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::TARGET_VELOCITY, 0x00, target_velocity as u32)
            .await
    }

    /// Stops the motor using the defined profile deceleration by setting target velocity to 0.
    pub async fn halt_velocity_movement(&mut self) -> Result<(), Epos4Error<E>> {
        self.move_with_velocity(0).await
    }

    /// Configures the parameters for the homing procedure.
    ///
    /// # Arguments
    /// * `switch_search_speed`: Speed for finding home switch (Object 0x6099/01).
    /// * `index_search_speed`: Speed for finding index pulse (Object 0x6099/02).
    /// * `acceleration`: Homing acceleration (Object 0x609A).
    /// * `offset`: Offset from found position (Object 0x30B1).
    pub async fn set_homing_params(
        &mut self,
        switch_search_speed: u32,
        index_search_speed: u32,
        acceleration: u32,
        offset: i32,
    ) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::HOMING_SPEEDS, 0x01, switch_search_speed)
            .await?;
        self.write_object(Self::HOMING_SPEEDS, 0x02, index_search_speed)
            .await?;
        self.write_object(Self::HOMING_ACCELERATION, 0x00, acceleration)
            .await?;
        self.write_object(Self::HOME_OFFSET_MOVE_DISTANCE, 0x00, offset as u32)
            .await?;
        Ok(())
    }

    /// Starts the homing procedure using the specified method (Object 0x6098).
    /// **Reference:** "EPOS4 Firmware Specification", page 29.
    pub async fn find_home(&mut self, method: HomingMethod) -> Result<(), Epos4Error<E>> {
        self.write_object(Self::HOMING_METHOD, 0x00, method as i8 as u32)
            .await?;
        // Start the homing procedure by setting bit 4 of the ControlWord
        let enable_op_word = 0x000F;
        let start_homing_word = enable_op_word | (1 << 4);
        self.write_object(Self::CONTROL_WORD, 0x00, start_homing_word)
            .await
    }

    /// Checks the status of the homing procedure. Poll this function after calling `find_home`.
    /// Based on StatusWord bits 12 (attained) and 13 (error).
    /// **Reference:** "EPOS4 Firmware Specification", page 30.
    pub async fn get_homing_status(&mut self) -> Result<HomingStatus, Epos4Error<E>> {
        let status_word = self.read_object(Self::STATUS_WORD, 0x00).await? as u16;
        Ok(HomingStatus {
            attained: (status_word & (1 << 12)) != 0,
            error: (status_word & (1 << 13)) != 0,
        })
    }

    // --- Data Acquisition ---

    /// Reads the current device status from the state machine.
    pub async fn get_status(&mut self) -> Result<EposStatus, Epos4Error<E>> {
        let answer = self.read_object(Self::STATUS_WORD, 0x00).await?;
        Ok((answer as u16).into())
    }

    /// Reads the current motor position (Object 0x6064).
    pub async fn get_current_position(&mut self) -> Result<i32, Epos4Error<E>> {
        self.read_object(Self::POSITION_ACTUAL_VALUE, 0x00)
            .await
            .map(|val| val as i32)
    }

    /// Reads the current motor velocity (Object 0x606C).
    pub async fn get_current_velocity(&mut self) -> Result<i32, Epos4Error<E>> {
        self.read_object(Self::VELOCITY_ACTUAL_VALUE, 0x00)
            .await
            .map(|val| val as i32)
    }

    /// Checks if the last commanded target has been reached (StatusWord bit 10).
    pub async fn target_reached(&mut self) -> Result<bool, Epos4Error<E>> {
        let status_word = self.read_object(Self::STATUS_WORD, 0x00).await?;
        let target_reached_bit = 1 << 10;
        Ok((status_word as u16 & target_reached_bit) == target_reached_bit)
    }

    /// Reads the raw average motor current (Object 0x30D1/01).
    ///
    /// NOTE: This returns a raw value in per mille of the motor's rated current.
    /// `Current [A] = (Raw Value / 1000.0) * Motor Rated Current [A]`
    pub async fn get_current_avg_raw(&mut self) -> Result<i16, Epos4Error<E>> {
        self.read_object(Self::CURRENT_ACTUAL_VALUES, 0x01)
            .await
            .map(|val| val as i16)
    }

    // --- Low-level Protocol Implementation ---

    /// Low-level function to write a 32-bit value to a specific object.
    async fn write_object(
        &mut self,
        index: u16,
        subindex: u8,
        data: u32,
    ) -> Result<(), Epos4Error<E>> {
        let data_words = [
            self.node_id as u16 | (index << 8),
            (index >> 8) | (subindex as u16) << 8,
            (data & 0xFFFF) as u16,
            (data >> 16) as u16,
        ];
        self.send_request_frame(0x68, &data_words).await?;
        let _response: [u16; 0] = self.read_response_frame().await?;
        Ok(())
    }

    /// Low-level function to read a 32-bit value from a specific object.
    async fn read_object(&mut self, index: u16, subindex: u8) -> Result<u32, Epos4Error<E>> {
        let data_words = [
            self.node_id as u16 | (index << 8),
            (index >> 8) | (subindex as u16) << 8,
        ];
        self.send_request_frame(0x60, &data_words).await?;
        let response: [u16; 2] = self.read_response_frame().await?;
        Ok(response[0] as u32 | (response[1] as u32) << 16)
    }

    async fn send_request_frame(&mut self, opcode: u8, data: &[u16]) -> Result<(), Epos4Error<E>> {
        let len = data.len() as u8;
        self.send_sync().await?;
        let header = ((len as u16) << 8) | (opcode as u16);
        let crc = Self::calculate_crc(header, data);
        self.send_word(header).await?;
        for &word in data {
            self.send_word(word).await?;
        }
        self.send_word(crc).await?;
        Ok(())
    }

    async fn read_response_frame<const L: usize>(&mut self) -> Result<[u16; L], Epos4Error<E>> {
        self.read_sync_sequence().await?;
        let header = self.read_word().await?;
        let opcode = (header & 0xFF) as u8;
        let len = (header >> 8) as usize;

        if len != L + 2 {
            // Response length must be data length (L) + error code length (2 words)
            return Err(Epos4Error::Framing);
        }
        if opcode != 0x00 {
            // Opcode in response should be 0x00 for success
            return Err(Epos4Error::Framing);
        }

        let mut error_code = [0u16; 2];
        error_code[0] = self.read_word().await?;
        error_code[1] = self.read_word().await?;

        let mut data_array = [0u16; L];
        for item in &mut data_array {
            *item = self.read_word().await?;
        }

        let received_crc = self.read_word().await?;

        if !Self::check_crc(header, &error_code, &data_array, received_crc) {
            return Err(Epos4Error::Framing);
        }

        let device_error = DeviceError::from(error_code);
        if device_error != DeviceError::NoError {
            return Err(Epos4Error::Device(device_error));
        }

        Ok(data_array)
    }

    async fn send_sync(&mut self) -> Result<(), Epos4Error<E>> {
        self.send_byte(Self::DATA_LINK_ESCAPE, false).await?;
        self.send_byte(Self::START_OF_TEXT, false).await?;
        Ok(())
    }

    async fn read_sync_sequence(&mut self) -> Result<(), Epos4Error<E>> {
        if self.read_byte(false).await? != Self::DATA_LINK_ESCAPE {
            return Err(Epos4Error::Framing);
        }
        if self.read_byte(false).await? != Self::START_OF_TEXT {
            return Err(Epos4Error::Framing);
        }
        Ok(())
    }

    async fn send_word(&mut self, word: u16) -> Result<(), Epos4Error<E>> {
        self.send_byte((word & 0xFF) as u8, true).await?;
        self.send_byte((word >> 8) as u8, true).await?;
        Ok(())
    }

    async fn read_word(&mut self) -> Result<u16, Epos4Error<E>> {
        let low_byte = self.read_byte(true).await?;
        let high_byte = self.read_byte(true).await?;
        Ok(u16::from_le_bytes([low_byte, high_byte]))
    }

    async fn send_byte(&mut self, byte: u8, stuffed: bool) -> Result<(), Epos4Error<E>> {
        if stuffed && byte == Self::DATA_LINK_ESCAPE {
            self.tx
                .write_all(&[Self::DATA_LINK_ESCAPE])
                .await
                .map_err(Epos4Error::Interface)?;
        }
        self.tx
            .write_all(&[byte])
            .await
            .map_err(Epos4Error::Interface)?;
        Ok(())
    }

    async fn read_byte(&mut self, stuffed: bool) -> Result<u8, Epos4Error<E>> {
        let mut byte_buf = [0u8];
        self.rx
            .read_exact(&mut byte_buf)
            .await
            .map_err(|x| match x {
                ReadExactError::UnexpectedEof => Epos4Error::Framing,
                ReadExactError::Other(x) => Epos4Error::Interface(x),
            })?;
        let mut byte = byte_buf[0];
        if stuffed && byte == Self::DATA_LINK_ESCAPE {
            self.rx
                .read_exact(&mut byte_buf)
                .await
                .map_err(|x| match x {
                    ReadExactError::UnexpectedEof => Epos4Error::Framing,
                    ReadExactError::Other(x) => Epos4Error::Interface(x),
                })?;
            byte = byte_buf[0];
        }
        Ok(byte)
    }

    fn calculate_crc(header: u16, data: &[u16]) -> u16 {
        let mut crc = 0u16;
        Self::update_crc(&mut crc, header);
        for &word in data {
            Self::update_crc(&mut crc, word);
        }
        Self::update_crc(&mut crc, 0u16);
        crc
    }

    fn check_crc(header: u16, error_code: &[u16], data: &[u16], target_crc: u16) -> bool {
        let mut crc = 0u16;
        Self::update_crc(&mut crc, header);
        for &word in error_code {
            Self::update_crc(&mut crc, word);
        }
        for &word in data {
            Self::update_crc(&mut crc, word);
        }
        Self::update_crc(&mut crc, target_crc);
        crc == 0
    }

    fn update_crc(crc: &mut u16, value: u16) {
        let mut shifter = 0x8000u16;
        while shifter > 0 {
            let carry = (*crc & 0x8000) != 0;
            *crc <<= 1;
            if (value & shifter) != 0 {
                *crc += 1;
            }
            if carry {
                *crc ^= 0x1021; // CRC-16-CCITT polynomial
            }
            shifter >>= 1;
        }
    }
}
