extern crate embedded_hal;
extern crate linux_embedded_hal;
#[macro_use]
extern crate nb;
extern crate apds9960;

use apds9960::{Apds9960, LightGain};
use linux_embedded_hal::I2cdev;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sensor = Apds9960::new(dev);
    sensor.enable().unwrap();
    
    // Set maximum light sensitivity
    sensor.set_light_gain(LightGain::X64).unwrap();
    
    // Longer integration time for better sensitivity (256 cycles = ~711ms)
    sensor.set_light_integration_time(0x00).unwrap();
    
    sensor.enable_proximity().unwrap();
    sensor.enable_light().unwrap();
    
    // Note: It's good practice to handle errors for enable and enable_proximity too,
    // but focusing on the proximity reading as requested.
    
    loop {
        match block!(sensor.read_proximity()) {
            Ok(p) => {
                println!("Proximity: {}", p);
            }
            Err(e) => {
                eprintln!("Error reading proximity: {:?}", e);
                // Optionally, you might want to break the loop or
                // attempt to reinitialize the sensor here, depending
                // on the error type and desired behavior.
            }
        }
        match block!(sensor.read_light()) {
            Ok(p) => {
                println!("Light: {} {} {} {}", p.blue, p.green, p.red, p.clear);
            }
            Err(e) => {
                eprintln!("Error reading Light: {:?}", e);
                // Optionally, you might want to break the loop or
                // attempt to reinitialize the sensor here, depending
                // on the error type and desired behavior.
            }
        }
    }
}