use i2cdev::core::I2CDevice;
#[cfg(target_os = "linux")]
use i2cdev::linux::*;

use utils::*;

struct CompensationParams {
    t1: u16,
    t2: i16,
    t3: i16,
    h1: u8,
    h2: i16,
    h3: u8,
    h4: i16,
    h5: i16,
    h6: i8,
    p1: u16,
    p2: i16,
    p3: i16,
    p4: i16,
    p5: i16,
    p6: i16,
    p7: i16,
    p8: i16,
    p9: i16,
}

impl CompensationParams {
    pub fn new() -> CompensationParams {
        CompensationParams {
            t1: 0,
            t2: 0,
            t3: 0,
            h1: 0,
            h2: 0,
            h3: 0,
            h4: 0,
            h5: 0,
            h6: 0,
            p1: 0,
            p2: 0,
            p3: 0,
            p4: 0,
            p5: 0,
            p6: 0,
            p7: 0,
            p8: 0,
            p9: 0,
        }
    }

    pub fn load(&mut self, dev: &mut LinuxI2CDevice) -> Result<(), LinuxI2CError> {
        self.t1 = read_unsigned_short(dev, 0x88)?;
        self.t2 = read_signed_short(dev, 0x8A)?;
        self.t3 = read_signed_short(dev, 0x8C)?;
        self.h1 = dev.smbus_read_byte_data(0xA1)?;
        self.h2 = read_signed_short(dev, 0xE1)?;
        self.h3 = dev.smbus_read_byte_data(0xE3)?;
        let e4 = dev.smbus_read_byte_data(0xE4)? as i16;
        let e5 = dev.smbus_read_byte_data(0xE5)?;
        let e6 = dev.smbus_read_byte_data(0xE6)? as i16;
        self.h4 = (e4 << 4) + ((e5 & 0x0F) as i16);
        self.h5 = (e6 << 4) + (((e5 & 0xF0) >> 4) as i16);
        self.h6 = dev.smbus_read_byte_data(0xE7)? as i8;
        self.p1 = read_unsigned_short(dev, 0x8E)?;
        self.p2 = read_signed_short(dev, 0x90)?;
        self.p3 = read_signed_short(dev, 0x92)?;
        self.p4 = read_signed_short(dev, 0x94)?;
        self.p5 = read_signed_short(dev, 0x96)?;
        self.p6 = read_signed_short(dev, 0x98)?;
        self.p7 = read_signed_short(dev, 0x9A)?;
        self.p8 = read_signed_short(dev, 0x9C)?;
        self.p9 = read_signed_short(dev, 0x9E)?;
        Ok(())
    }

    pub fn fine_resolution_temp(&self, uncomp_t: u32) -> i32 {
        let t1 = self.t1 as f64;

        let x1 = ((uncomp_t as f64) / 16384.0 - t1 / 1024.0) * (self.t2 as f64);
        let x2_ = (uncomp_t as f64) / 131072.0 - t1 / 8192.0;
        let x2 = (x2_ * x2_) * (self.t3 as f64);
        (x1 + x2) as i32
    }

    pub fn compensated_temp(&self, uncomp_t: u32) -> f32 {
        let tf = ((self.fine_resolution_temp(uncomp_t) * 5 + 128) >> 8) as f32;
        tf / 100.0
    }

    pub fn compensated_humidity(&self, uncomp_h: u32, uncomp_t: u32) -> f64 {
        let t_fine = self.fine_resolution_temp(uncomp_t);
        let x1 = (t_fine as f64) - 76800.0;
        let x2 = ((self.h4 as f64) * 64.0) + ((self.h5 as f64) / 16384.0) * x1;
        let x3 = (uncomp_h as f64) - x2;
        let x4 = (self.h2 as f64) / 65536.0;
        let x5 = 1.0 + ((self.h3 as f64) / 67108864.0 * x1);
        let x6 = 1.0 + ((self.h6 as f64) / 67108864.0) * x1 * x5;
        let x7 = x3 * x4 * (x5 * x6);
        let humidity = x7 * (1.0 - (self.h1 as f64) * x7 / 524288.0);

        humidity
    }

    pub fn compensated_pressure(&self, uncomp_p: u32, uncomp_t: u32) -> f64 {
        const MAX: f64 = 110000.0;
        const MIN: f64 = 30000.0;

        let t_fine = self.fine_resolution_temp(uncomp_t) as f64;
        let mut x1 = t_fine / 2.0 - 64000.0;
        let mut x2 = x1 * x1 * (self.p6 as f64) / 32768.0;
        x2 = x2 + x1 * (self.p5 as f64) * 2.0;
        x2 = x2 / 4.0 + (self.p4 as f64) * 65536.0;
        let x3 = (self.p3 as f64) * x1 * x1 / 524288.0;
        x1 = (x3 + (self.p2 as f64) * x1) / 524288.0;
        x1 = (1.0 + x1 / 32768.0) * (self.p1 as f64);

        // To avoid zero-division error.
        if x1 == 0.0 {
            return MIN;
        }

        let mut pressure = 1048576.0 - (uncomp_p as f64);
        pressure = (pressure - (x2 / 4096.0)) * 6250.0 / x1;
        x1 = (self.p9 as f64) * pressure * pressure / 2147483648.0;
        x2 = pressure * (self.p8 as f64) / 32768.0;
        pressure = pressure + (x1 + x2 + (self.p7 as f64)) / 16.0;

        if pressure < MIN {
            return MIN;
        } else if pressure > MAX {
            return MAX;
        } else {
            return pressure;
        }
    }
}

#[derive(Clone)]
pub enum Mode {
    Sleep,
    Force,
    Normal,
}

impl Mode {
    fn to_raw(&self) -> u8 {
        match self {
            &Mode::Sleep => 0,
            &Mode::Force => 1,
            &Mode::Normal => 3,
        }
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
    device: LinuxI2CDevice,
    config: Config,
    params: CompensationParams,
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
                            self.config.mode.to_raw();
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

    pub fn humidity(&mut self) -> Result<f64, LinuxI2CError> {
        let raw_t = self.raw_temperature()?;
        let raw_h = self.raw_humidity()?;
        Ok(self.params.compensated_humidity(raw_h, raw_t))
    }

    pub fn pressure(&mut self) -> Result<f64, LinuxI2CError> {
        let raw_t = self.raw_temperature()?;
        let raw_p = self.raw_pressure()?;
        Ok(self.params.compensated_pressure(raw_p, raw_t))
    }
}
