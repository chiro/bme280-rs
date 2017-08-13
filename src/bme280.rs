extern crate i2cdev;

use i2cdev::core::I2CDevice;
#[cfg(target_os = "linux")]
use i2cdev::linux::*;

use utils::*;

pub struct CompensationParams {
    pub t1: u16,
    pub t2: i16,
    pub t3: i16,
}

impl CompensationParams {
    pub fn new() -> CompensationParams {
        CompensationParams {
            t1: 0,
            t2: 0,
            t3: 0,
        }
    }

    pub fn load(&mut self, dev: &mut LinuxI2CDevice) -> Result<(), LinuxI2CError> {
        self.t1 = read_unsigned_short(dev, 0x88)?;
        self.t2 = read_signed_short(dev, 0x8A)?;
        self.t3 = read_signed_short(dev, 0x8C)?;
        Ok(())
    }

    pub fn fine_resolution_temp(&self, uncomp_t: u32) -> i32 {
        let uncomp_ = uncomp_t as i32;
        let t1_ = self.t1 as i32;
        let t2_ = self.t2 as i32;
        let t3_ = self.t3 as i32;

        let x1 = (((uncomp_ >> 3) - (t1_ << 1)) * t2_) >> 11;
        let x2 = (((((uncomp_ >> 4) - t1_) * ((uncomp_ >> 4) - t1_)) >> 12) * t3_) >> 14;
        x1 + x2
    }

    pub fn compensated_temp(&self, uncomp_t: u32) -> f32 {
        let tf = ((self.fine_resolution_temp(uncomp_t) * 5 + 128) >> 8) as f32;
        tf / 100.0
    }
}

#[derive(Clone)]
pub enum Mode {
    Sleep,
    Force,
    Normal,
}

fn to_value(mode: Mode) -> u8 {
    match mode {
        Mode::Sleep => 0,
        Mode::Force => 1,
        Mode::Normal => 3,
    }
}

pub struct Config {
    pub mode: Mode,
    pub oversampling_temperature: u8,
    pub oversampling_pressure: u8,
    pub oversampling_humidity: u8,
    pub standby_time: u8,
    pub iir_filter: u8,
    pub spi3w_enabled: bool,
}

pub struct BME280 {
    pub device: LinuxI2CDevice,
    config: Config,
    pub params: CompensationParams,
}

impl BME280 {
    pub fn new(dev: LinuxI2CDevice, config: Config) -> Result<BME280, LinuxI2CError> {
        let params = CompensationParams::new();
        let mut bme280 = BME280 {
            device: dev,
            config: config,
            params: params,
        };
        bme280.params.load(&mut bme280.device)?;
        bme280.initialize()?;
        Ok(bme280)
    }

    fn initialize(&mut self) -> Result<(), LinuxI2CError> {
        let ctrl_hum_reg = self.config.oversampling_humidity;
        let ctrl_meas_reg = (self.config.oversampling_temperature << 5) |
                            (self.config.oversampling_pressure << 2) |
                            to_value(self.config.mode.clone());
        let config_reg = (self.config.standby_time << 5) | (self.config.iir_filter << 2) |
                         (self.config.spi3w_enabled as u8);

        // ctrl_hum_reg has to be written before ctrl_meas_reg
        self.device.smbus_write_byte_data(0xF2, ctrl_hum_reg)?;
        self.device.smbus_write_byte_data(0xF4, ctrl_meas_reg)?;
        self.device.smbus_write_byte_data(0xF5, config_reg)
    }

    pub fn raw_pressure(&mut self) -> Result<u32, LinuxI2CError> {
        let v = self.device.smbus_read_i2c_block_data(0xF7, 3)?;
        let p0 = v[0] as u32;
        let p1 = v[1] as u32;
        let p2 = v[2] as u32;
        Ok((p0 << 12) + (p1 << 4) + (p2 >> 4))
    }

    pub fn raw_temperature(&mut self) -> Result<u32, LinuxI2CError> {
        let v = self.device.smbus_read_i2c_block_data(0xFA, 3)?;
        let t0 = v[0] as u32;
        let t1 = v[1] as u32;
        let t2 = v[2] as u32;
        Ok((t0 << 12) + (t1 << 4) + (t2 >> 4))
    }

    pub fn raw_humidity(&mut self) -> Result<u32, LinuxI2CError> {
        let v = self.device.smbus_read_i2c_block_data(0xFD, 2)?;
        let h1 = v[0] as u32;
        let h2 = v[1] as u32;
        Ok((h1 << 8) + h2)
    }

    pub fn temperature(&mut self) -> Result<f32, LinuxI2CError> {
        let raw_value = self.raw_temperature()?;
        Ok(self.params.compensated_temp(raw_value))
    }
}
