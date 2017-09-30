//! # bme280-rs
//!
//! This crate provides you a way to access bme280 via Linux I2C interface.
//!
//! # Examples
//! ```
//! let config: Config = Config {
//!     mode: Mode::Force,
//!     oversampling_temperature: Oversampling::X1,
//!     oversampling_pressure: Oversampling::X1,
//!     oversampling_humidity: Oversampling::X2,
//!     standby_time: StandbyTime::Ms1,
//!     filter_coeff: IIRFilterCoeff::OFF,
//!     spi3w_enabled: false,
//! };
//! let dev = LinuxI2CDevice::new("/dev/i2c-0", 119).unwrap();
//! let mut bme280 = BME280::new(dev, config).unwrap();
//! bme280.oneshot_measure().unwrap();
//! println!("{}", bme280.temperature().unwrap());
//! println!("{:.2}", bme280.humidity().unwrap());
//! println!("{:.2}", bme280.pressure().unwrap() / 100.0);
//! ```

extern crate i2cdev;
extern crate nix;

mod bme280;
mod utils;

pub use bme280::*;
