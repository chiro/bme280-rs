use std::{thread, time};

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

    pub fn compensated_temp(&self, uncomp_t: u32) -> f64 {
        const MAX: f64 = 85.0;
        const MIN: f64 = -40.0;

        let tf = ((self.fine_resolution_temp(uncomp_t) * 5 + 128) >> 8) as f64;
        (tf / 100.0).max(MIN).min(MAX)
    }

    pub fn compensated_humidity(&self, uncomp_h: u32, uncomp_t: u32) -> f64 {
        const MAX: f64 = 100.0;
        const MIN: f64 = 0.0;

        let t_fine = self.fine_resolution_temp(uncomp_t);
        let x1 = (t_fine as f64) - 76800.0;
        let x2 = ((self.h4 as f64) * 64.0) + ((self.h5 as f64) / 16384.0) * x1;
        let x3 = (uncomp_h as f64) - x2;
        let x4 = (self.h2 as f64) / 65536.0;
        let x5 = 1.0 + ((self.h3 as f64) / 67108864.0 * x1);
        let x6 = 1.0 + ((self.h6 as f64) / 67108864.0) * x1 * x5;
        let x7 = x3 * x4 * (x5 * x6);
        let humidity = x7 * (1.0 - (self.h1 as f64) * x7 / 524288.0);

        humidity.max(MIN).min(MAX)
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

        pressure.min(MAX).max(MIN)
    }
}

/// Power mode of BME280.
pub enum Mode {
    /// No operation mode, all registeres are accessible.
    Sleep,
    /// Perform one-shot measurement.
    Force,
    /// Do perpetual cycling of measurements.
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

/// Oversampling options for humidity, temperature, pressure measurement.
pub enum Oversampling {
    No,
    X1,
    X2,
    X4,
    X8,
    X16,
}

impl Oversampling {
    fn to_raw(&self) -> u8 {
        match self {
            &Oversampling::No => 0,
            &Oversampling::X1 => 1,
            &Oversampling::X2 => 2,
            &Oversampling::X4 => 3,
            &Oversampling::X8 => 4,
            &Oversampling::X16 => 5,
        }
    }
}

/// Stand-by time options for Normal mode.
/// This option doesn't have any effect on Sleep mode and Force mode.
#[allow(non_camel_case_types)]
pub enum StandbyTime {
    /// 1 ms.
    Ms1,
    /// 62.5 ms.
    Ms62_5,
    /// 125 ms.
    Ms125,
    /// 250 ms.
    Ms250,
    /// 500 ms.
    Ms500,
    /// 1000 ms.
    Ms1000,
    /// 10 ms.
    Ms10,
    /// 20 ms.
    Ms20,
}

impl StandbyTime {
    fn to_raw(&self) -> u8 {
        match self {
            &StandbyTime::Ms1 => 0,
            &StandbyTime::Ms62_5 => 1,
            &StandbyTime::Ms125 => 2,
            &StandbyTime::Ms250 => 3,
            &StandbyTime::Ms500 => 4,
            &StandbyTime::Ms1000 => 5,
            &StandbyTime::Ms10 => 6,
            &StandbyTime::Ms20 => 7,
        }
    }
}

/// Coefficient for internal IIR filter for temperature and pressure measurement.
pub enum IIRFilterCoeff {
    OFF,
    X2,
    X4,
    X8,
    X16,
}

impl IIRFilterCoeff {
    fn to_raw(&self) -> u8 {
        match self {
            &IIRFilterCoeff::OFF => 0,
            &IIRFilterCoeff::X2 => 1,
            &IIRFilterCoeff::X4 => 2,
            &IIRFilterCoeff::X8 => 3,
            &IIRFilterCoeff::X16 => 4,
        }
    }
}

/// Device configuration.
pub struct Config {
    pub mode: Mode,
    pub oversampling_temperature: Oversampling,
    pub oversampling_pressure: Oversampling,
    pub oversampling_humidity: Oversampling,
    pub standby_time: StandbyTime,
    pub filter_coeff: IIRFilterCoeff,
    pub spi3w_enabled: bool,
}

/// Device.
pub struct BME280 {
    device: LinuxI2CDevice,
    config: Config,
    params: CompensationParams,
}

impl BME280 {
    /// Return new BME280 device with the provided configuration.
    pub fn new(dev: LinuxI2CDevice, config: Config) -> Result<BME280, LinuxI2CError> {
        let params = CompensationParams::new();
        let mut bme280 = BME280 {
            device: dev,
            config: config,
            params: params,
        };
        bme280.check_device_id()?;
        bme280.soft_reset()?;
        bme280.params.load(&mut bme280.device)?;
        bme280.initialize()?;
        Ok(bme280)
    }

    fn check_device_id(&mut self) -> Result<(), LinuxI2CError> {
        use i2cdev::linux::LinuxI2CError::Nix;

        const CHIP_ID: u8 = 0x60;
        let id = self.device.smbus_read_byte_data(0xD0)?;
        if CHIP_ID != id {
            return Err(Nix(nix::Error::Sys(nix::errno::Errno::ENXIO)));
        }
        Ok(())
    }

    fn initialize(&mut self) -> Result<(), LinuxI2CError> {
        let ctrl_hum_reg = self.config.oversampling_humidity.to_raw();
        let ctrl_meas_reg = (self.config.oversampling_temperature.to_raw() << 5)
            | (self.config.oversampling_pressure.to_raw() << 2)
            | self.config.mode.to_raw();
        let config_reg = (self.config.standby_time.to_raw() << 5)
            | (self.config.filter_coeff.to_raw() << 2)
            | (self.config.spi3w_enabled as u8);

        // ctrl_hum_reg has to be written before ctrl_meas_reg
        self.device.smbus_write_byte_data(0xF2, ctrl_hum_reg)?;
        self.device.smbus_write_byte_data(0xF4, ctrl_meas_reg)?;
        self.device.smbus_write_byte_data(0xF5, config_reg)
    }

    /// Do soft reset.
    pub fn soft_reset(&mut self) -> Result<(), LinuxI2CError> {
        const SOFT_RESET_CMD: u8 = 0xB6;
        self.device.smbus_write_byte_data(0xE0, SOFT_RESET_CMD)?;
        // Startup time is 2ms as per the data sheet.
        thread::sleep(time::Duration::from_millis(2));
        Ok(())
    }

    /// Do one-shot measurement.
    /// After calling this method, you can get values by |temperature|,
    /// |humidity| and |pressure| methods.
    pub fn oneshot_measure(&mut self) -> Result<(), LinuxI2CError> {
        let ctrl_meas_reg = self.device.smbus_read_byte_data(0xF4)?;
        let new_value = (ctrl_meas_reg & 0xFC) | Mode::Force.to_raw();
        let end_value = (ctrl_meas_reg & 0xFC) | Mode::Sleep.to_raw();

        self.device.smbus_write_byte_data(0xF4, new_value)?;
        // Wait for measurement end at most 20 ms.
        for _ in 1..10 {
            let v = self.device.smbus_read_byte_data(0xF4)?;
            if v == end_value {
                break;
            }
            thread::sleep(time::Duration::from_millis(2));
        }
        Ok(())
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

    /// Gets current temperature in degree Celsius.
    pub fn temperature(&mut self) -> Result<f64, LinuxI2CError> {
        let raw_value = self.raw_temperature()?;
        Ok(self.params.compensated_temp(raw_value))
    }

    /// Gets current humidity in percent.
    pub fn humidity(&mut self) -> Result<f64, LinuxI2CError> {
        let raw_t = self.raw_temperature()?;
        let raw_h = self.raw_humidity()?;
        Ok(self.params.compensated_humidity(raw_h, raw_t))
    }

    /// Gets current pressure in Pa.
    pub fn pressure(&mut self) -> Result<f64, LinuxI2CError> {
        let raw_t = self.raw_temperature()?;
        let raw_p = self.raw_pressure()?;
        Ok(self.params.compensated_pressure(raw_p, raw_t))
    }
}
