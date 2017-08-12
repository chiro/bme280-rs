extern crate i2cdev;

use i2cdev::core::I2CDevice;
#[cfg(target_os = "linux")]
use i2cdev::linux::*;

pub fn read_unsigned_short(dev: &mut LinuxI2CDevice, address: u8) -> Result<u16, LinuxI2CError> {
    let values = dev.smbus_read_i2c_block_data(address, 2)?;
    let v0 = values[0] as u16;
    let v1 = values[1] as u16;
    Ok(v0 + (v1 << 8))
}

pub fn read_signed_short(dev: &mut LinuxI2CDevice, address: u8) -> Result<i16, LinuxI2CError> {
    Ok(read_unsigned_short(dev, address)? as i16)
}
