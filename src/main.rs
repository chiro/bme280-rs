extern crate docopt;
extern crate i2cdev;
#[macro_use]
extern crate serde_derive;

use docopt::Docopt;

use std::{thread, time};

use i2cdev::core::I2CDevice;
#[cfg(target_os = "linux")]
use i2cdev::linux::*;

mod utils;
use utils::*;

mod bme280;
use bme280::*;

const USAGE: &'static str = "
Reading BME280 sensor value

Usage:
  bme280 <device> [--address <addr>]
  bme280 (-h | --help)
  bme280 (-v | --version)

Options:
  -h --help    Show this help text.
  --address <addr>     I2C device address [default: 119] (=0x77)
  -v --version    Show version.
";

const BME280_DEFAULT_ADDRESS: u16 = 0x76;

#[derive(Debug, Deserialize)]
struct Args {
    arg_device: String,
    flag_address: Option<u16>,
    flag_version: bool,
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("This program can run only on Linux")
}

fn read_uncomp_pressure(dev: &mut LinuxI2CDevice) -> Result<u32, LinuxI2CError> {
    let press_vec = dev.smbus_read_i2c_block_data(0xF7, 3)?;
    let p1: u32 = press_vec[0] as u32;
    let p2: u32 = press_vec[1] as u32;
    let p3: u32 = press_vec[2] as u32;
    Ok((p1 << 12) + (p2 << 4) + (p3 >> 4))
}

fn read_uncomp_temperature(dev: &mut LinuxI2CDevice) -> Result<u32, LinuxI2CError> {
    let temp_vec = dev.smbus_read_i2c_block_data(0xFA, 3)?;
    let t1: u32 = temp_vec[0] as u32;
    let t2: u32 = temp_vec[1] as u32;
    let t3: u32 = temp_vec[2] as u32;
    Ok((t1 << 12) + (t2 << 4) + (t3 >> 4))
}

fn read_uncomp_humidity(dev: &mut LinuxI2CDevice) -> Result<u32, LinuxI2CError> {
    let humid_vec = dev.smbus_read_i2c_block_data(0xFD, 2)?;
    let h1: u32 = humid_vec[0] as u32;
    let h2: u32 = humid_vec[1] as u32;
    Ok((h1 << 8) + h2)
}

#[cfg(target_os = "linux")]
fn main() {
    let args: Args = Docopt::new(USAGE).and_then(|d| d.deserialize()).unwrap_or_else(|e| e.exit());

    if args.flag_version {
        println!("bme280 {}", env!("CARGO_PKG_VERSION"));
        return;
    }

    let address = args.flag_address.unwrap_or(BME280_DEFAULT_ADDRESS);
    let config: Config = Config {
        mode: Mode::Force,
        oversampling_temperature: 1,
        oversampling_pressure: 1,
        oversampling_humidity: 1,
        standby_time: 5,
        iir_filter: 0,
        spi3w_enabled: false,
    };

    let mut dev = LinuxI2CDevice::new(args.arg_device, address).unwrap();
    let mut bme280 = BME280::new(dev, config).unwrap();

    // Sleep while measureing finishes.
    let ten_millis = time::Duration::from_millis(10);
    thread::sleep(ten_millis);

    let uncomp_press = read_uncomp_pressure(&mut bme280.device).unwrap();
    let uncomp_temperature = read_uncomp_temperature(&mut bme280.device).unwrap();
    let uncomp_humidity = read_uncomp_humidity(&mut bme280.device).unwrap();
    println!("uncomp_temp = {}, uncomp_press = {}, uncomp_humid = {}",
             uncomp_temperature,
             uncomp_press,
             uncomp_humidity);

    let temperature = bme280.params.compensated_temp(uncomp_temperature);
    println!("temperature = {}", temperature);
}
