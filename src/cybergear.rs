use std::{f32::consts::PI, io::Write, marker::PhantomData};
use serial::prelude::*;

pub trait FloatRange {
    fn min() -> f32;
    fn max() -> f32;
}

pub struct Torque {}
impl FloatRange for Torque {
    fn min() -> f32 { -12f32 }
    fn max() -> f32 { 12f32 }
}

pub struct Position {}
impl FloatRange for Position {
    fn min() -> f32 { -4f32 * PI }
    fn max() -> f32 { 4f32 * PI }
}

pub struct AngularVelocity {}
impl FloatRange for AngularVelocity {
    fn min() -> f32 { -30f32 }
    fn max() -> f32 { 30f32 }
}

pub struct KD {}
impl FloatRange for KD {
    fn min() -> f32 { 0f32 }
    fn max() -> f32 { 5f32 }
}

pub struct KP {}
impl FloatRange for KP {
    fn min() -> f32 { 0f32 }
    fn max() -> f32 { 500f32 }
}

pub struct CyberGearFloat<T: FloatRange> {
    pub _type: PhantomData<T>,
    pub value: f32,
    pub can_value: u16
}

// make numbers out of the cybergear values
impl<T: FloatRange> CyberGearFloat<T> {
    pub fn new(value: f32) -> Result<Self, Box<dyn std::error::Error>> {
        let min = T::min();
        let max = T::max();
        if value < min || value > max {
            Err(format!("Value out of range: {} not in [{}, {}]", value, min, max).into())
        } else {
            let can_value = ((value - min) / (max - min) * (u16::MAX as f32)) as u16;
            Ok(CyberGearFloat::<T> {
                _type: PhantomData::<T> {},
                value: value,
                can_value: can_value,
            })
        }
    }

    pub fn new_from_can(can_value: u16) -> Self {
        let fraction = (can_value as f32) / (u16::MAX as f32);
        let scaled_fraction = (T::max() - T::min()) * fraction;
        let value = scaled_fraction + T::min();
        CyberGearFloat::<T> {
            _type: PhantomData::<T> {},
            value: value,
            can_value: can_value,
        }
    }

    pub fn new_from_can_large(can_value: u32) -> Self {
        let fraction = (can_value as f32) / (u32::MAX as f32);
        let scaled_fraction = (T::max() - T::min()) * fraction;
        let value = scaled_fraction + T::min();
        CyberGearFloat::<T> {
            _type: PhantomData::<T> {},
            value: value,
            can_value: can_value as u16,
        }
    }
}

pub struct MotorStatus {
    pub id: u32,
    pub current_motor_can_id: u8,
    pub not_calibrated: bool,
    pub hall_encoding_fault: bool,
    pub magnetic_encoding_barrier: bool,
    pub over_temperature: bool,
    pub overcurrent: bool,
    pub undervoltage_fault: bool,
    pub mode_status: String,
    pub current_angle: CyberGearFloat<Position>,
    pub current_angular_velocity: CyberGearFloat<AngularVelocity>,
    pub current_torque: CyberGearFloat<Torque>,
    pub current_temperature: f32, // Assuming no CyberGearFloat type for temperature
}

impl MotorStatus {
    pub fn from_can_frame(frame: Box<dyn CanFramable>) -> Result<Self, Box<dyn std::error::Error>> {
        let id = frame.get_id();
        let current_motor_can_id = ((id >> 8) & 0xFF) as u8;
        let not_calibrated = ((id >> 21) & 0x1) != 0;
        let hall_encoding_fault = ((id >> 20) & 0x1) != 0;
        let magnetic_encoding_barrier = ((id >> 19) & 0x1) != 0;
        let over_temperature = ((id >> 18) & 0x1) != 0;
        let overcurrent = ((id >> 17) & 0x1) != 0;
        let undervoltage_fault = ((id >> 16) & 0x1) != 0;
        let mode_status = match (id >> 22) & 0x3 {
            0 => "Reset mode [reset]".to_string(),
            1 => "Cali mode [standard]".to_string(),
            2 => "Motor mode [run]".to_string(),
            _ => { panic!("unknown motor mode") },
        };

        // it's annoying but the floats in the motor status are in big endian
        let data = frame.get_data();
        let current_angle = CyberGearFloat::<Position>::new_from_can(u16::from_be_bytes([data[0], data[1]]));
        let current_angular_velocity = CyberGearFloat::<AngularVelocity>::new_from_can(u16::from_be_bytes([data[2], data[3]]));
        let current_torque = CyberGearFloat::<Torque>::new_from_can(u16::from_be_bytes([data[4], data[5]]));
        let current_temperature = (u16::from_be_bytes([data[6], data[7]]) as f32) / 10f32;

        Ok(MotorStatus {
            id,
            current_motor_can_id,
            not_calibrated,
            hall_encoding_fault,
            magnetic_encoding_barrier,
            over_temperature,
            overcurrent,
            undervoltage_fault,
            mode_status,
            current_angle,
            current_angular_velocity,
            current_torque,
            current_temperature,
        })
    }
}

impl std::fmt::Display for MotorStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MotorStatus {{\n\
             \tid: {:X},\n\
             \tcurrent_motor_can_id: {},\n\
             \tnot_calibrated: {},\n\
             \thall_encoding_fault: {},\n\
             \tmagnetic_encoding_barrier: {},\n\
             \tover_temperature: {},\n\
             \tovercurrent: {},\n\
             \tundervoltage_fault: {},\n\
             \tmode_status: {},\n\
             \tcurrent_angle: {},\n\
             \tcurrent_angular_velocity: {},\n\
             \tcurrent_torque: {},\n\
             \tcurrent_temperature: {:.1}°C\n\
             }}",
            self.id,
            self.current_motor_can_id,
            self.not_calibrated,
            self.hall_encoding_fault,
            self.magnetic_encoding_barrier,
            self.over_temperature,
            self.overcurrent,
            self.undervoltage_fault,
            self.mode_status,
            self.current_angle.value,
            self.current_angular_velocity.value,
            self.current_torque.value,
            self.current_temperature
        )
    }
}

pub struct SLCan(pub slcan::CanSocket::<serial::SystemPort>);

impl SLCan {
    pub fn default() -> Result<Self, Box<dyn std::error::Error>> {
        let default_port = "/dev/tty.usbmodem101"; // This is a common default port for SLCAN devices on Unix-like systems
        let mut port = serial::open(default_port)?;
        port.set_timeout(std::time::Duration::from_secs(1))?;
        let mut can_socket = slcan::CanSocket::new(port);                
        can_socket.open(slcan::BitRate::Setup1Mbit).expect("set the bitrate");
        Ok(SLCan(can_socket))
    }
}

impl Drop for SLCan {
    fn drop(&mut self) {
        if let Err(e) = self.0.close() {
            eprintln!("Failed to close the CAN socket: {}", e);
        }
    }
}



pub trait CanFramable {
    fn get_id(&self) -> u32;
    fn get_data(&self) -> [u8; 8];
}

pub struct SLCanFrame(pub slcan::CanFrame);
impl CanFramable for SLCanFrame {
    fn get_id(&self) -> u32 {
        match self.0.id {
            slcan::Id::Standard(id) => id.as_raw() as u32,
            slcan::Id::Extended(id) => id.as_raw(),
        }
    }

    fn get_data(&self) -> [u8; 8] {
        self.0.data
    }
}

pub trait CanBusable {
    fn write(&mut self, id: u32, data: &[u8]) -> Result<(), Box<dyn std::error::Error>>;
    fn read(&mut self) -> Result<Box<dyn CanFramable>, Box<dyn std::error::Error>>;
}

impl CanBusable for SLCan {
    fn write(&mut self, id: u32, data: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        let can_id = slcan::Id::Extended(slcan::ExtendedId::new(id).unwrap());
        self.0.write(can_id, data).unwrap();
        Ok(())
    }

    fn read(&mut self) -> Result<Box<dyn CanFramable>, Box<dyn std::error::Error>> {
        let frame = self.0.read().unwrap();
        Ok(Box::new(SLCanFrame(frame)))
    }
}

pub trait CyberGearCanBusable: CanBusable {
    fn get_host_can_id(&self) -> u8;

    fn set_motor_param_bytes(
        &mut self,
        motor_id: u8,
        param_index: u16,
        param_bytes: [u8; 4]
    ) {
        let can_id = u32::from_le_bytes([motor_id, self.get_host_can_id(), 0x00, 0x12]);
        let param_index = param_index.to_le_bytes();
        self.write(can_id, &[param_index[0], param_index[1], 0, 0, param_bytes[0], param_bytes[1], param_bytes[2], param_bytes[3]]).expect("failed to write motor param");
        let output = self.read().expect("failed to read motor param");
    }

    fn get_motor_param_bytes(
        &mut self,
        motor_id: u8,
        param_index: u16
    ) -> Result<[u8; 8], Box<dyn std::error::Error>> {
        let can_id = u32::from_le_bytes([motor_id, self.get_host_can_id(), 0x00, 0x11]);
        let param_index = param_index.to_le_bytes();
        self.write(can_id, &[param_index[0], param_index[1], 0, 0, 0, 0, 0, 0]).expect("failed to write motor param");
        let output = self.read().expect("failed to read motor param");
        let output = output.get_data();        
        Ok(output)
    }

    fn enable_motor(
        &mut self,
        motor_id: u8
    ) -> Result<MotorStatus, Box<dyn std::error::Error>> {    
        let can_id = u32::from_le_bytes([motor_id, self.get_host_can_id(), 0x00, 0x03]);
        let data: [u8; 8] = [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0];
        self.write(can_id, &data).expect("Failed to write to CAN bus");
        let frame = self.read().expect("Failed to read from CAN bus");
        MotorStatus::from_can_frame(frame)
    }

    fn disable_motor(
        &mut self,
        motor_id: u8
    ) -> Result<MotorStatus, Box<dyn std::error::Error>> {
        let can_id = u32::from_le_bytes([motor_id, self.get_host_can_id(), 0x00, 0x04]);
        let data: [u8; 8] = [0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0];
        self.write(can_id, &data)?;
        let frame = self.read().expect("failed to read from canbus");
        MotorStatus::from_can_frame(frame)
    }

    fn go_to_position(
        &mut self,
        motor_id: u8,
        position: CyberGearFloat<Position>,
        torque: CyberGearFloat<Torque>,
        angular_velocity: CyberGearFloat<AngularVelocity>,
        kp: CyberGearFloat<KP>,
        kd: CyberGearFloat<KD>
    ) -> Result<MotorStatus, Box<dyn std::error::Error>> {
        // TODO: Pretty sure that this is wrong and that we need to be using to_be_bytes
        let torque_bytes = torque.can_value.to_le_bytes();
        let position_bytes = position.can_value.to_le_bytes();
        let angular_velocity_bytes = angular_velocity.can_value.to_le_bytes();
        let kp_bytes = kp.can_value.to_le_bytes();
        let kd_bytes = kd.can_value.to_le_bytes();
        let can_id = u32::from_le_bytes([
            motor_id,
            torque_bytes[0],
            torque_bytes[1],
            1u8,
        ]);
        let data_le_bytes = [
            position_bytes[0],
            position_bytes[1],
            angular_velocity_bytes[0],
            angular_velocity_bytes[1],
            kp_bytes[0],
            kp_bytes[1],
            kd_bytes[0], 
            kd_bytes[1]
        ];
        self.write(can_id, &data_le_bytes)?;
        let frame = self.read()?;
        MotorStatus::from_can_frame(frame)
    }
}

impl CyberGearCanBusable for SLCan {
    fn get_host_can_id(&self) -> u8 {
        return 0;
    }
}
