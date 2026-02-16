#![allow(dead_code)]
#![allow(clippy::needless_range_loop)]
/*
    This is the implementation for the RSBL85-24 servos. Thanks to Domenic Nebiker for helping me build this thing
    Those Servos are controlled using Half-Duplex RS-485. I currently believe that the following is the
    Register map: This is stolen from the Arduino implementation of the Servo driver hat they use
    I believe we need to use the SMS type Registers / Protocols, as I can find SCS servos that use potentiometers
    and are not connected via RS-485 but instead half-duplex UART over a single wire.
    (Also M kinda corresponds to magnetic, and as we are using servos with magnetic encoders instead of potentiometers this is appropriate)
    The SCS Servo registers only differ slightly, and it should be possible to use the RSBL Servos with either implementation.

    This here is a first approximation of the program that will be ultimately used to control these servos and is currently only intended
    for testing purposes.

*/
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::with_timeout;
use embedded_utils::fmt::*;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

// ADDRESSING STUFF
const HEADER: u8 = 0xFF; //must be duplicated at the beginning of every message;
const BROADCAST: u8 = 0xFE;
pub const LEFT: u8 = 2;
pub const RIGHT: u8 = 3;
pub const RSBL_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(5);

//<editor-fold desc="Declarations for Servo Registers">
// EEPROM (read only) //

/// Firmware major version number
/// initial value = 3
const FIRMWARE_MAJOR: u8 = 0x00;

/// Firmware sub version number
/// initial value = 6
const FIRMWARE_MINOR: u8 = 0x01;

/// Servo main version number
/// initial value = 9
const SERVO_MAIN_VERSION: u8 = 0x03;

/// Servo sub version number
/// initial value = 3
const SERVO_MINOR_VERSION: u8 = 0x04;

// EEPROM (read and write) //

/// ID
/// Unique identification code on the bus. Duplicate ID number is not allowed on the same bus, 254 (0xFE) is the broadcast ID, broadcast does not return a reply packet.
/// initial value = 1
const ID: u8 = 0x05;

/// Baud rate
/// 0-7 represents baud rate as follows: 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400
/// initial value = 0
const BAUD_RATE: u8 = 0x06;

/// Return delay
/// The minimum unit is 2µs, and the maximum set return delay is 254 * 2 = 508µs.
/// initial value = 0
const RETURN_DELAY: u8 = 0x07;

/// Response status level
/// 0: except for read instruction and Ping instruction, other instructions do not return a reply packet;
/// 1: Returns a reply packet for all instructions.
/// initial value = 1
const RESPONSE_STATUS_LEVEL: u8 = 0x08;

/// Minimum Angle Limitation
/// Set the minimum limit of motion stroke, the value is less than the maximum angle limit, and this value is 0 when the multi-cycle absolute position control is carried out.
/// initial value = 0
const MIN_ANGLE_LIMIT_H: u8 = 0x09;
const MIN_ANGLE_LIMIT_L: u8 = 0x0A;

/// Maximum Angle Limitation
/// Set the maximum limit of motion stroke, which is greater than the minimum angle limit, and the value is 0 when the multi-turn absolute position control is adopted.
/// initial value = 4095
const MAX_ANGLE_LIMIT_H: u8 = 0x0B;
const MAX_ANGLE_LIMIT_L: u8 = 0x0C;

/// Maximum Temperature Limit
/// The maximum operating temperature limit, if set to 70, the maximum temperature is 70°C, and the setting accuracy is 1°C.
/// initial value = 70
const MAX_TEMPERATURE: u8 = 0x0D;

/// Maximum input voltage
/// If the maximum input voltage is set to 80, the maximum working voltage is limited to 8.0V and the setting accuracy is 0.1V.
/// initial value = 80
const MAX_INPUT_VOLTAGE: u8 = 0x0E;

/// Minimum input voltage
/// If the minimum input voltage is set to 40, the minimum working voltage is limited to 4.0V and the setting accuracy is 0.1V.
/// initial value = 40
const MIN_INPUT_VOLTAGE: u8 = 0x0F;

/// Maximum torque
/// Set the maximum output torque limit of the servo, and set 1000 = 100% * locked torque.
/// Power on assigned to address 48 torque limit.
/// initial value = 1000
const MAX_TORQUE_H: u8 = 0x10;
const MAX_TORQUE_L: u8 = 0x11;

/// Phase
/// Special function byte, which cannot be modified without special requirements.
/// initial value = 12
const PHASE: u8 = 0x12;

/// Unloading condition
/// Bit0-Bit5 corresponding bits are set to enable corresponding protection.
/// initial value = 44
const UNLOADING_CONDITION: u8 = 0x13;

/// LED Alarm condition
/// The corresponding bit of temperature, current, angle, overload, or voltage sensor is set to 0 to disable the alarm.
/// initial value = 47
const LED_ALARM_CONDITION: u8 = 0x14;

/// P Proportionality coefficient
/// Proportional factor of control motor.
/// initial value = 32
const P_PROPORTIONALITY_COEFFICIENT: u8 = 0x15;

/// D Differential coefficient
/// Differential coefficient of control motor.
/// initial value = 32
const D_DIFFERENTIAL_COEFFICIENT: u8 = 0x16;

/// I Integral coefficient
/// Integral coefficient of control motor.
/// initial value = 0
const I_INTEGRAL_COEFFICIENT: u8 = 0x17;

/// Minimum startup force
/// Set the minimum output starting torque of the servo. 1000 = 100% * locked torque.
/// initial value = 16
const MIN_STARTUP_FORCE_H: u8 = 0x18;
const MIN_STARTUP_FORCE_L: u8 = 0x19;

/// Clockwise insensitive area
/// The minimum unit is a minimum resolution angle.
/// initial value = 1
const CW_INSENSITIVE_AREA: u8 = 0x1A;

/// Counterclockwise insensitive region
/// The minimum unit is a minimum resolution angle.
/// initial value = 1
const CCW_INSENSITIVE_REGION: u8 = 0x1B;

/// Protection current
/// The maximum current can be set at 3255mA.
/// initial value = 500
const PROTECTION_CURRENT_H: u8 = 0x1C;
const PROTECTION_CURRENT_L: u8 = 0x1D;

/// Angular resolution
/// The amplification factor of the minimum resolution angle (degree/step).
/// initial value = 1
const ANGULAR_RESOLUTION: u8 = 0x1E;

/// Position correction
/// Bit11 is the direction bit, indicating positive and negative directions.
/// initial value = 0
const POSITION_CORRECTION_H: u8 = 0x1F;
const POSITION_CORRECTION_L: u8 = 0x20;

/// Operation mode
/// 0: Position servo mode
/// 1: Constant speed mode (controlled by parameter 0x2E, bit 15 is direction bit)
/// 2: PWM open-loop speed regulation mode
/// 3: Step servo mode (step progress by parameter 0x2A, bit 15 is direction bit)
/// initial value = 0
const OPERATION_MODE: u8 = 0x21;

/// Protective torque
/// After entering overload protection, if set to 20, means 20% of max torque.
/// initial value = 20
const PROTECTIVE_TORQUE: u8 = 0x22;

/// Protection time
/// Timing time when current load exceeds overload torque and remains.
/// initial value = 200
const PROTECTION_TIME: u8 = 0x23;

/// Overload torque
/// Max torque threshold for starting overload protection.
/// initial value = 80
const OVERLOAD_TORQUE: u8 = 0x24;

/// Speed closed loop P proportional coefficient
/// In motor constant speed mode (mode 1), the speed loop proportional coefficient.
/// initial value = 10
const SPEED_CLOSED_LOOP_P: u8 = 0x25;

/// Overcurrent protection time
/// Max setting is 254 * 10ms = 2540ms.
/// initial value = 200
const OVER_CURRENT_PROTECTION_TIME: u8 = 0x26;

/// Velocity closed loop I integral coefficient
/// initial value = 10
const VELOCITY_CLOSED_LOOP_I: u8 = 0x27;

// SRAM (read and write) //

/// Torque switch
/// initial value = 0
const TORQUE_SWITCH: u8 = 0x28;

/// Acceleration
/// initial value = 0
const ACCELERATION: u8 = 0x29;

/// Target location
/// initial value = 0
const TARGET_LOCATION_H: u8 = 0x2A;
const TARGET_LOCATION_L: u8 = 0x2B;

/// Running time
/// initial value = 0
const RUNNING_TIME_H: u8 = 0x2C;
const RUNNING_TIME_L: u8 = 0x2D;

/// Running speed
/// initial value = 0
const RUNNING_SPEED_H: u8 = 0x2E;
const RUNNING_SPEED_L: u8 = 0x2F;

/// Torque limit
/// initial value = 1000
const TORQUE_LIMIT_H: u8 = 0x30;
const TORQUE_LIMIT_L: u8 = 0x31;

/// Lock mark
/// initial value = 0
const LOCK_MARK: u8 = 0x37;

/// Current location
/// initial value = 0
const CURRENT_LOCATION_H: u8 = 0x38;
const CURRENT_LOCATION_L: u8 = 0x39;

/// Current speed
/// initial value = 0
const CURRENT_SPEED_H: u8 = 0x3A;
const CURRENT_SPEED_L: u8 = 0x3B;

/// Current load
/// initial value = 0
const CURRENT_LOAD_H: u8 = 0x3C;
const CURRENT_LOAD_L: u8 = 0x3D;

/// Current voltage
/// initial value = 0
const CURRENT_VOLTAGE: u8 = 0x3E;

/// Current temperature
/// initial value = 0
const CURRENT_TEMPERATURE: u8 = 0x3F;

/// Asynchronous write flag
/// initial value = 0
const ASYNC_WRITE_FLAG: u8 = 0x40;

/// Servo status
/// initial value = 0
const SERVO_STATUS: u8 = 0x41;

/// Mobile sign
/// initial value = 0
const MOBILE_SIGN: u8 = 0x42;

/// Current current
/// initial value = 0
const CURRENT_CURRENT_H: u8 = 0x45;
const CURRENT_CURRENT_L: u8 = 0x46;
//</editor-fold>

//<editor-fold desc="Servo Command List">
//INSTRUCTIONS FOR SERVO
const PING: u8 = 0x01; // Query working status | Parameter length = 0
const READ_DATA: u8 = 0x02; //Query the character in the control table | Parameter length = 2
const WRITE_DATA: u8 = 0x03; //Write the character into the control table | Parameter length >= 1
const REGWRITE_DATA: u8 = 0x04; //Similar to WRITE DATA, but the control character does not act immediately after writing until ACTION. | Parameter length >= 2
const ACTION: u8 = 0x05; //Triggering the action of REG WRITE operation | Parameter length = 0, suitable for broadcast
const SYNCREAD_DATA: u8 = 0x82; // Query multiple servos at the same time. | Parameter length >= 3
const SYNCWRITE_DATA: u8 = 0x83; //Controlling multiple servos at the same time | Parameter length >= 2
const RESET: u8 = 0x06; //Reset the control table to the factory value | Parameter length = 0

//</editor-fold>

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RsblError {
    FailedWrite,
    FailedRead,
    InvalidAngle,
    InternalError,
    WrongChecksum,
    WrongFormat,
    InvalidID,
    TimeoutError,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RsblData {
    pub angle: i32,
    pub speed: i16,
    pub load: u16,
    pub voltage: u8,
    pub temperature: u8,
    pub moving: bool,
    pub current: u16,
}
impl RsblData {
    pub fn new(
        angle: i32,
        speed: i16,
        load: u16,
        voltage: u8,
        temperature: u8,
        moving: bool,
        current: u16,
    ) -> Self {
        Self {
            angle,
            speed,
            load,
            voltage,
            temperature,
            moving,
            current,
        }
    }
}

pub struct RsblServo<'d> {
    tx: UartTx<'d, Async>,
    rx: RingBufferedUartRx<'d>,
    dir: Output<'static>,
    left_pos: i32,
    right_pos: i32,
}

pub fn uart_config() -> usart::Config {
    // missing @Lennard:
    // Baud rate setting
    // Start/Stop bits config

    let mut config: usart::Config = Default::default();
    config.baudrate = 1000000; //115200
    config.stop_bits = usart::StopBits::STOP1;
    config.parity = usart::Parity::ParityNone;
    config.data_bits = usart::DataBits::DataBits8;
    config.detect_previous_overrun = false;
    config.assume_noise_free = true;
    config
}

// generally missing: function to make two u8 to one u16, and vice versa
impl<'d> RsblServo<'d> {
    ///generate new RsblServo instance
    pub fn new(
        handle: usart::Uart<'d, Async>,
        dir: Output<'static>,
        rx_dma_buf: &'static mut [u8],
    ) -> Self {
        let (tx, rx) = handle.split();
        let rx = rx.into_ring_buffered(rx_dma_buf);
        let left_pos = 0;
        let right_pos = 0;

        Self {
            tx,
            rx,
            dir,
            left_pos,
            right_pos,
        }
    }

    pub async fn startup_sequence(&mut self) -> Result<(), RsblError> {
        let mut error = None;

        // handle the weird data that is sent during startup
        self.reset();
        match self.clear_ringbuffer().await {
            Ok(_) => {}
            Err(e) => {
                error!(
                    "Error while clearing ringbuffer during steering startup: {}",
                    e
                );
                error = Some(e);
            }
        }
        self.reset();

        match self.move_steps(LEFT, 0, 0xFFFE).await {
            Ok(_) => {}
            Err(e) => {
                error!(
                    "Error while locking left steering motors in startup sequence: {}",
                    e
                );
                error = Some(e);
            }
        }
        match self.move_steps(RIGHT, 0, 0xFFFE).await {
            Ok(_) => {}
            Err(e) => {
                error!(
                    "Error while locking right steering motors in startup sequence: {}",
                    e
                );
                error = Some(e);
            }
        }
        match error {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    pub fn reset(&mut self) {
        let config = uart_config();
        self.rx.set_config(&config).unwrap();
        let config = uart_config();
        self.tx.set_config(&config).unwrap();
    }
    /* ===== High Level Function Implementations for single servos ===== */
    /* These only properly work with our servo hardware + software configs, may have unexpected behaviour otherwise */

    /// set up a new servo. NEVER USE THIS IN FINAL FIRMWARE! IT WILL (MAYBE) KILL THINGS
    /// Servo is set up to use step servo mode and config is saved over power cycles
    /// Lock instruction might not work correctly right now, but that should be fixable by swapping the data between 1 and 0
    /// also try to set the voltage to actual 24V and also try to play around with the torque settings
    pub async fn setup_servo(&mut self, original_id: u8, new_id: u8) -> Result<(), RsblError> {
        //ID is given. First, we disable write protection. Then we set the new id
        self.write_data(original_id, LOCK_MARK, &[0x00]).await?;

        //this should work...
        self.write_data(original_id, ID, &[new_id]).await?;

        //now we configure the servo so that the correct config is saved:
        // we want to use operating mode 3 and no angle restriction for the servos to work properly
        self.write_data(new_id, OPERATION_MODE, &[0x03]).await?;
        self.write_data(new_id, MIN_ANGLE_LIMIT_H, &[0x00, 0x00, 0x00, 0x00])
            .await?;

        //reset lock instruction
        self.write_data(new_id, LOCK_MARK, &[0x01]).await?;

        Ok(())
    }

    /// move both servos present in the rocket with one function call, simplify error handling maybe?
    pub async fn steer_parachutes(
        &mut self,
        left_target_position: i32,
        right_target_position: i32,
    ) -> Result<(), RsblError> {
        self.move_absolute(LEFT, left_target_position).await?;
        self.move_absolute(RIGHT, right_target_position).await?;
        Ok(())
    }

    /// read out data from both servos to get current feedback, also include target position for tracking purposes
    /// This function is a helper function that expands on the functionality of read_sensor_data.
    /// It computes the actual servo positions, as in the current operating mode (step servo mode) the servo only returns the amount of steps which it has to
    /// do until it reaches the previously provided step count.
    pub async fn read_steering_data(&mut self, id: u8) -> Result<Option<RsblData>, RsblError> {
        let data = self.read_sensor_data(id).await?;

        match id {
            LEFT => {
                if let Some(mut data) = data {
                    data.angle = self.left_pos - data.angle;
                    Ok(Some(data))
                } else {
                    Ok(None)
                }
            }
            RIGHT => {
                if let Some(mut data) = data {
                    data.angle = self.right_pos - data.angle;
                    Ok(Some(data))
                } else {
                    Ok(None)
                }
            }
            _ => Err(RsblError::InvalidID),
        }
    }

    /// moves the servo to a step count relative to the zero-position (zero position is startup position) and tracks that movement
    /// currently, if there is a power loss, we will still lose accuracy but not entire position data
    /// only accepts IDs LEFT and RIGHT, as those are the only ones we will use
    pub async fn move_absolute(&mut self, id: u8, absolute_position: i32) -> Result<(), RsblError> {
        let curr_pos = match id {
            LEFT => self.left_pos,
            RIGHT => self.right_pos,
            _ => return Err(RsblError::InvalidID),
        };
        let mut error: Option<RsblError> = None;

        let mut steps = absolute_position - curr_pos;

        while steps.abs() > 32766 {
            if steps < 0 {
                steps += 32766;
                match self.move_steps(id, -32766, 0xFFFE).await {
                    Ok(()) => {}
                    Err(e) => {
                        error = Some(e);
                    }
                }
            } else {
                steps -= 32766;
                match self.move_steps(id, 32766, 0xFFFE).await {
                    Ok(()) => {}
                    Err(e) => {
                        error = Some(e);
                    }
                }
            }
        }

        match self.move_steps(id, steps as i16, 0xFFFE).await {
            Ok(()) => {}
            Err(e) => {
                error = Some(e);
            }
        }

        match id {
            LEFT => {
                self.left_pos = absolute_position;
            }
            RIGHT => {
                self.right_pos = absolute_position;
            }
            _ => return Err(RsblError::InvalidID),
        }
        if let Some(e) = error {
            return Err(e);
        }
        Ok(())
    }

    /* ===== Middle Level Function Implementations for single servos ===== */
    /* These only properly work with our servo hardware configs, may have unexpected behaviour otherwise */

    /// set a specific angle (in steps) to the servo id, using speed
    /// general config: 4096 steps = 1 rotation
    pub async fn move_steps(&mut self, id: u8, angle: i16, speed: u16) -> Result<(), RsblError> {
        //check that angle is a valid value and fits into an u15 (yes, u15, not u16 as the last bit is needed for direction setting):
        if angle.abs() > 32766 {
            return Err(RsblError::InvalidAngle);
        }

        //generate the correct and final value for the angle
        let angle_reg: u16 = if angle < 0 {
            let angle = -angle;
            (1 << 15) + angle as u16
        } else {
            angle as u16
        };

        //put all the data into the buffer
        let mut data = [0u8; 6];

        //enter all the data correctly
        let mut bytes = angle_reg.to_le_bytes();
        data[0..2].copy_from_slice(&bytes);

        bytes = speed.to_le_bytes();
        data[4..6].copy_from_slice(&bytes);

        //ATTENTION HERE! Names might change. Please make sure to always use the correct variable
        self.write_data(id, TARGET_LOCATION_H, &data).await?;

        Ok(())
    }

    /// Returns all Sensor Data from the Motor of id that can be read out.
    /// The idea behind this absolutely awful data structure in the return messae is the following:
    /// When we get data, the return is a Ok(Some(data)), indicating that no error has occured and that data is present.
    /// If no servo is connected the return is a Ok(None), indicating that the UART is working as expected, however no return message was detected, indicating no connection
    /// If there is an error, I return Err(e) in order to indicate that sth is wrong with the UART or sth else.
    pub async fn read_sensor_data(&mut self, id: u8) -> Result<Option<RsblData>, RsblError> {
        //get current pos, current speed, current load, current voltage, current temperature
        let mut buf = [0u8; 13];
        match self.read_data(id, CURRENT_LOCATION_H, &mut buf).await {
            Ok(()) => {
                //process data

                //process angle
                let mut angle_reg = (buf[1] as u16) << 8 | buf[0] as u16;

                let angle: i32 = if (angle_reg >> 15) == 1 {
                    angle_reg -= 1 << 15;
                    -(angle_reg as i32)
                } else {
                    angle_reg as i32
                };

                //process speed
                //actually compute a meaningful speed value...
                //prob works similarly to the angle value, however that needs to be tested
                let mut speed_reg = (buf[3] as u16) << 8 | buf[2] as u16;
                let speed: i16 = if (speed_reg >> 15) == 1 {
                    speed_reg -= 1 << 15;
                    -(speed_reg as i16)
                } else {
                    speed_reg as i16
                };

                //process load
                //I don't know and I have now idea how to interpret this number
                let load = (buf[5] as u16) << 8 | buf[4] as u16;

                //voltage
                //compute actual voltage???
                let voltage = buf[6];

                //temperature
                let temperature = buf[7];

                //currently moving flag

                let moving = buf[10] > 0;

                //current consumption
                let current_consumption = (buf[12] as u16) << 8 | buf[11] as u16;
                let rsbl_data = RsblData::new(
                    angle,
                    speed,
                    load,
                    voltage,
                    temperature,
                    moving,
                    current_consumption,
                );
                Ok(Some(rsbl_data))
            }
            Err(e) => match e {
                RsblError::TimeoutError => Ok(None),
                _ => Err(e),
            },
        }
    }

    /* ===== Low Level Function Implementations ===== */
    ///ping RSBL Servo id
    pub async fn ping(&mut self, id: u8) -> Result<(), RsblError> {
        self.write_to_servo(id, PING, None, None).await?;

        //get response. As no data is returned, we need to give the read function None
        self.read_from_servo(id, None).await?;

        Ok(())
    }

    ///read data starting at start_address with length buf.len() into buf from Servo id
    pub async fn read_data(
        &mut self,
        id: u8,
        start_address: u8,
        buf: &mut [u8],
    ) -> Result<(), RsblError> {
        //send read command to servos

        //generate write parameter buffer. Only includes the length of the buffer we want to read in
        let write_params = [buf.len() as u8];
        self.write_to_servo(id, READ_DATA, Some(start_address), Some(&write_params))
            .await?;

        //read returned data into buf, and perform some checks with the returned data
        self.read_from_servo(id, Some(buf)).await?;
        Ok(())
    }

    ///write array data to Servo id registers, starting at start_data
    pub async fn write_data(
        &mut self,
        id: u8,
        start_address: u8,
        data: &[u8],
    ) -> Result<(), RsblError> {
        //write data to servo starting at adress start_adress
        self.write_to_servo(id, WRITE_DATA, Some(start_address), Some(data))
            .await?;

        //read the return frame
        self.read_from_servo(id, None).await?;
        Ok(())
    }

    /* ===== Lowest Level Function Implementations for directly reading/writing the UART ===== */
    /// write instruction to the servo
    async fn write_to_servo(
        &mut self,
        id: u8,
        instruction: u8,
        reg: Option<u8>,
        data: Option<&[u8]>,
    ) -> Result<(), RsblError> {
        let checksum;

        //this always stays the same, no matter what we want to write
        let mut header = [HEADER, HEADER, id, 0, instruction];

        //there is some code duplication due to timing issues. AFAIK the UART frame needs to be timed
        //almost perfectly, and I could not get it to work otherwise
        match reg {
            //if we have given a register to write to, then we also have data. Handle that case here
            Some(register) => {
                //info!("Some data in data buf: {}", data);
                //get data length
                let data = data.unwrap_or(&[]);
                //set length of instruction: instruction + reg + data.len() + checksum
                let length = (data.len() + 3) as u8;
                header[3] = length;
                checksum = self.calc_checksum(id, length, instruction, Some(register), Some(data));

                //actually write stuff to the servo
                self.dir.set_high();
                self.tx
                    .write(&header)
                    .await
                    .map_err(|_| RsblError::FailedWrite)?;
                self.tx
                    .write(&[register])
                    .await
                    .map_err(|_| RsblError::FailedWrite)?;
                self.tx
                    .write(data)
                    .await
                    .map_err(|_| RsblError::FailedWrite)?;
                self.tx
                    .write(&[checksum])
                    .await
                    .map_err(|_| RsblError::FailedWrite)?;
                self.tx.flush().await.map_err(|_| RsblError::FailedWrite)?;
                self.dir.set_low();
                Ok(())
            }
            //if we do not have a register to write to, then we also do not have data. handle that here
            None => {
                //info!("No data in data buf");
                //if no register parameter is given, then length will always be 2 (instruction + checksum)

                let length: u8 = 2;
                header[3] = length;
                checksum = self.calc_checksum(id, length, instruction, None, None);

                if let Some(_data) = data {
                    return Err(RsblError::WrongFormat);
                }

                let buf = [
                    header[0], header[1], header[2], header[3], header[4], checksum,
                ];

                //actually write stuff to the servo
                //written in a blocking way because async takes too much time, but for sending it still needs to be tested
                self.dir.set_high();
                self.tx
                    .write(&buf)
                    .await
                    .map_err(|_| RsblError::FailedWrite)?;
                self.tx.flush().await.map_err(|_| RsblError::FailedWrite)?;
                self.dir.set_low();

                Ok(())
            }
        }
    }

    ///reads data from the RSBL servos, and gives back all relevant data in the buf. Checks for errors with checksum and servo status. Data is still saved to buf
    async fn read_from_servo(&mut self, id: u8, buf: Option<&mut [u8]>) -> Result<(), RsblError> {
        //no return if broadcast is used
        if id == BROADCAST {
            return Ok(());
        }

        let mut data = [0u8];
        let buf = buf.unwrap_or(&mut []);

        //ensure that transceiver can read
        self.dir.set_low();

        //generate header arrays, only needed for some checks
        let mut header: [u8; 5] = [0; 5];
        let mut checksum: [u8; 1] = [0];

        // We need to read into the buffers byte-wise as otherwise at the end of the ring buffer it does not close within the read operation and
        // sets the part of the array exceeding the ring buffer to 0, which fucks up everything
        for i in 0..header.len() {
            match with_timeout(RSBL_TIMEOUT, self.rx.read(&mut data)).await {
                Ok(Ok(_)) => {}
                Ok(Err(e)) => {
                    error!("{}", e);
                    return Err(RsblError::FailedRead);
                }
                Err(_) => return Err(RsblError::TimeoutError),
            }
            header[i] = data[0];
        }

        // I may get empty buffers (e.g. ping, write), and this is so that I can use this function for every instance

        if !buf.is_empty() {
            for i in 0..buf.len() {
                match with_timeout(RSBL_TIMEOUT, self.rx.read(&mut data)).await {
                    Ok(Ok(_)) => {}
                    Ok(Err(_)) => return Err(RsblError::FailedRead),
                    Err(_) => return Err(RsblError::TimeoutError),
                }
                buf[i] = data[0];
            }
        }

        match with_timeout(RSBL_TIMEOUT, self.rx.read(&mut checksum)).await {
            Ok(Ok(_)) => {}
            Ok(Err(_)) => return Err(RsblError::FailedRead),
            Err(_) => return Err(RsblError::TimeoutError),
        }
        //info!("return frame : {:x}, {:x}, {:x}", header, buf, checksum);

        // I honestly don't believe we need to do this anymore, however I will keep it in here for completeness's sake
        let _ = self.clear_ringbuffer().await;

        #[allow(arithmetic_overflow)]
        if buf.len() + 2 != header[3] as usize {
            error!("sth wrong");
            return Err(RsblError::WrongFormat);
        }
        //check data if everything went according to plan. Data is already saved in the buffers, so
        // even if this fails data can still be extracted
        if checksum[0] == self.calc_checksum(header[2], header[3], header[4], None, Some(buf)) {
            //Check Servo internal stuff
            if header[4] == 0 {
                Ok(())
            } else {
                Err(RsblError::InternalError)
            }
        } else {
            Err(RsblError::WrongChecksum)
        }
    }

    pub async fn clear_ringbuffer(&mut self) -> Result<(), RsblError> {
        let buf = &mut [0u8];
        loop {
            match with_timeout(RSBL_TIMEOUT, self.rx.read(buf)).await {
                Ok(Ok(_)) => {
                    info!("rx buffer had sth in it: {:X}", buf)
                }
                Ok(Err(_e)) => return Err(RsblError::InternalError),
                Err(_e) => {
                    info!("flushed rx buffer, returning");
                    return Ok(());
                }
            }
        }
    }

    ///calculate checksum for servo frames
    fn calc_checksum(
        &self,
        id: u8,
        length: u8,
        instruction: u8,
        register: Option<u8>,
        data: Option<&[u8]>,
    ) -> u8 {
        let data = data.unwrap_or(&[0u8]);
        let register = register.unwrap_or(0);

        let mut sum: u16 = data.iter().map(|&b| b as u16).sum();

        sum += id as u16 + length as u16 + instruction as u16 + register as u16;

        !(sum as u8)
    }
}
