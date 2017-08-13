extern crate docopt;
extern crate i2cdev;
#[macro_use]
extern crate serde_derive;

use docopt::Docopt;

use std::{thread, time};

#[cfg(target_os = "linux")]
use i2cdev::linux::*;

mod utils;

mod bme280;
use bme280::*;

const USAGE: &'static str = "
Reading BME280 sensor value

Usage:
  bme280 <device> [--address <addr>] [--temperature] [--humidity]
  bme280 (-h | --help)
  bme280 (-v | --version)

Options:
  -h --help    Show this help text.
  --address <addr>     I2C device address [default: 119] (=0x77)
  --temperature    Show temperature.
  --humidity    Show humidity
  -v --version    Show version.
";

const BME280_DEFAULT_ADDRESS: u16 = 0x76;

#[derive(Debug, Deserialize)]
struct Args {
    arg_device: String,
    flag_address: Option<u16>,
    flag_version: bool,
    flag_temperature: bool,
    flag_humidity: bool,
}

#[cfg(not(target_os = "linux"))]
fn main() {
    println!("This program can run only on Linux")
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
        oversampling_humidity: 2,
        standby_time: 5,
        iir_filter: 0,
        spi3w_enabled: false,
    };

    let dev = LinuxI2CDevice::new(args.arg_device, address).unwrap();
    let mut bme280 = BME280::new(dev, config).unwrap();

    // Sleep while measureing finishes.
    let millis = time::Duration::from_millis(100);
    thread::sleep(millis);

    if args.flag_temperature {
        println!("{}", bme280.temperature().unwrap());
    }
    if args.flag_humidity {
        println!("{:.2}", bme280.humidity().unwrap());
    }
}
