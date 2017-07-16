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
  bme280 <device>
  bme280 (-h | --help)
  bme280 (-v | --version)

Options:
  -h --help    Show this help text.
  -v --version    Show version.
";

#[derive(Debug, Deserialize)]
struct Args {
    arg_device: String,
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

    println!("<device> = {}", args.arg_device);
}
