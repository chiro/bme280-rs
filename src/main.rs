extern crate docopt;
extern crate i2cdev;
#[macro_use]
extern crate serde_derive;

use docopt::Docopt;

#[cfg(target_os = "linux")]
use i2cdev::linux::*;

const USAGE: &'static str = "
Reading BME280 sensor value

Usage:
  bme280 <device> [--address <addr>]
  bme280 (-h | --help)
  bme280 (-v | --version)

Options:
  -h --help    Show this help text.
  --address <addr>     I2C device address [default: 0x77(=119)]
  -v --version    Show version.
";

const BME280_DEFAULT_ADDRESS: u16 = 0x77;

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

#[cfg(target_os = "linux")]
fn main() {
    let args: Args = Docopt::new(USAGE).and_then(|d| d.deserialize()).unwrap_or_else(|e| e.exit());

    if args.flag_version {
        println!("bme280 {}", env!("CARGO_PKG_VERSION"));
        return;
    }

    let address = args.flag_address.unwrap_or(BME280_DEFAULT_ADDRESS);
    println!("<device> = {}, <addr> = 0x{:x}", args.arg_device, address)
}
