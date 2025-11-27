//! Gain control and LED configuration types and methods.
//!
//! This module provides type-safe enums for controlling sensor gains and LED settings,
//! along with methods to configure them on the APDS9960 sensor.

use hal::i2c;
use {Apds9960, Error, Register};

/// Proximity gain multiplier
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProximityGain {
    /// 1x gain
    X1 = 0,
    /// 2x gain
    X2 = 1,
    /// 4x gain
    X4 = 2,
    /// 8x gain
    X8 = 3,
}

/// Ambient light/color gain multiplier
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LightGain {
    /// 1x gain
    X1 = 0,
    /// 4x gain
    X4 = 1,
    /// 16x gain
    X16 = 2,
    /// 64x gain
    X64 = 3,
}

/// Gesture gain multiplier
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GestureGain {
    /// 1x gain
    X1 = 0,
    /// 2x gain
    X2 = 1,
    /// 4x gain
    X4 = 2,
    /// 8x gain
    X8 = 3,
}

/// LED drive current
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LedDrive {
    /// 100 mA
    Ma100 = 0,
    /// 50 mA
    Ma50 = 1,
    /// 25 mA
    Ma25 = 2,
    /// 12.5 mA
    Ma12_5 = 3,
}

/// LED boost current
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LedBoost {
    /// 100%
    Percent100 = 0,
    /// 150%
    Percent150 = 1,
    /// 200%
    Percent200 = 2,
    /// 300%
    Percent300 = 3,
}

impl<I2C, E> Apds9960<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Set proximity sensor gain
    pub fn set_proximity_gain(&mut self, gain: ProximityGain) -> Result<(), Error<E>> {
        let mut control = self.read_register(Register::CONTROL)?;
        control = (control & 0xF3) | ((gain as u8) << 2);
        self.write_register(Register::CONTROL, control)
    }

    /// Set ambient light/color sensor gain
    pub fn set_light_gain(&mut self, gain: LightGain) -> Result<(), Error<E>> {
        let mut control = self.read_register(Register::CONTROL)?;
        control = (control & 0xFC) | (gain as u8);
        self.write_register(Register::CONTROL, control)
    }

    /// Set gesture sensor gain
    pub fn set_gesture_gain(&mut self, gain: GestureGain) -> Result<(), Error<E>> {
        let mut gconf2 = self.read_register(Register::GCONF2)?;
        gconf2 = (gconf2 & 0x9F) | ((gain as u8) << 5);
        self.write_register(Register::GCONF2, gconf2)
    }

    /// Get gesture sensor gain
    pub fn get_gesture_gain(&mut self) -> Result<GestureGain, Error<E>> {
        let gconf2 = self.read_register(Register::GCONF2)?;
        let value = (gconf2 >> 5) & 0x03;
        Ok(match value {
            0 => GestureGain::X1,
            1 => GestureGain::X2,
            2 => GestureGain::X4,
            _ => GestureGain::X8,
        })
    }

    /// Set LED drive current for proximity and ALS
    pub fn set_led_drive(&mut self, drive: LedDrive) -> Result<(), Error<E>> {
        let mut control = self.read_register(Register::CONTROL)?;
        control = (control & 0x3F) | ((drive as u8) << 6);
        self.write_register(Register::CONTROL, control)
    }

    /// Set gesture LED drive current
    pub fn set_gesture_led_drive(&mut self, drive: LedDrive) -> Result<(), Error<E>> {
        let mut gconf2 = self.read_register(Register::GCONF2)?;
        gconf2 = (gconf2 & 0xE7) | ((drive as u8) << 3);
        self.write_register(Register::GCONF2, gconf2)
    }

    /// Get gesture LED drive current
    pub fn get_gesture_led_drive(&mut self) -> Result<LedDrive, Error<E>> {
        let gconf2 = self.read_register(Register::GCONF2)?;
        let value = (gconf2 >> 3) & 0x03;
        Ok(match value {
            0 => LedDrive::Ma100,
            1 => LedDrive::Ma50,
            2 => LedDrive::Ma25,
            _ => LedDrive::Ma12_5,
        })
    }

    /// Set LED boost current
    pub fn set_led_boost(&mut self, boost: LedBoost) -> Result<(), Error<E>> {
        let mut config2 = self.read_register(Register::CONFIG2)?;
        config2 = (config2 & 0xCF) | ((boost as u8) << 4);
        self.write_register(Register::CONFIG2, config2)
    }

    /// Set proximity pulse count and length
    /// 
    /// * `pulses`: Number of pulses (0-63, actual pulses = value + 1)
    /// * `length`: Pulse length (0=4us, 1=8us, 2=16us, 3=32us)
    pub fn set_proximity_pulse(&mut self, pulses: u8, length: u8) -> Result<(), Error<E>> {
        let value = ((length & 0x03) << 6) | (pulses & 0x3F);
        self.write_register(Register::PPULSE, value)
    }

    /// Set gesture pulse count and length
    /// 
    /// * `pulses`: Number of pulses (0-63, actual pulses = value + 1)
    /// * `length`: Pulse length (0=4us, 1=8us, 2=16us, 3=32us)
    pub fn set_gesture_pulse(&mut self, pulses: u8, length: u8) -> Result<(), Error<E>> {
        let value = ((length & 0x03) << 6) | (pulses & 0x3F);
        self.write_register(Register::GPULSE, value)
    }

    /// Set gesture wait time between gesture detection cycles
    ///
    /// * 0 = 0 ms
    /// * 1 = 2.8 ms
    /// * 2 = 5.6 ms
    /// * 3 = 8.4 ms
    /// * 4 = 14.0 ms
    /// * 5 = 22.4 ms
    /// * 6 = 30.8 ms
    /// * 7 = 39.2 ms
    pub fn set_gesture_wait_time(&mut self, time: u8) -> Result<(), Error<E>> {
        let mut gconf2 = self.read_register(Register::GCONF2)?;
        gconf2 = (gconf2 & 0xF8) | (time & 0x07);
        self.write_register(Register::GCONF2, gconf2)
    }

    /// Get gesture wait time between gesture detection cycles
    pub fn get_gesture_wait_time(&mut self) -> Result<u8, Error<E>> {
        let gconf2 = self.read_register(Register::GCONF2)?;
        Ok(gconf2 & 0x07)
    }

    /// Get proximity sensor gain
    pub fn get_proximity_gain(&mut self) -> Result<ProximityGain, Error<E>> {
        let control = self.read_register(Register::CONTROL)?;
        let value = (control >> 2) & 0x03;
        Ok(match value {
            0 => ProximityGain::X1,
            1 => ProximityGain::X2,
            2 => ProximityGain::X4,
            _ => ProximityGain::X8,
        })
    }

    /// Get ambient light/color sensor gain
    pub fn get_light_gain(&mut self) -> Result<LightGain, Error<E>> {
        let control = self.read_register(Register::CONTROL)?;
        let value = control & 0x03;
        Ok(match value {
            0 => LightGain::X1,
            1 => LightGain::X4,
            2 => LightGain::X16,
            _ => LightGain::X64,
        })
    }

    /// Get LED drive current for proximity and ALS
    pub fn get_led_drive(&mut self) -> Result<LedDrive, Error<E>> {
        let control = self.read_register(Register::CONTROL)?;
        let value = (control >> 6) & 0x03;
        Ok(match value {
            0 => LedDrive::Ma100,
            1 => LedDrive::Ma50,
            2 => LedDrive::Ma25,
            _ => LedDrive::Ma12_5,
        })
    }

    /// Get LED boost current
    pub fn get_led_boost(&mut self) -> Result<LedBoost, Error<E>> {
        let config2 = self.read_register(Register::CONFIG2)?;
        let value = (config2 >> 4) & 0x03;
        Ok(match value {
            0 => LedBoost::Percent100,
            1 => LedBoost::Percent150,
            2 => LedBoost::Percent200,
            _ => LedBoost::Percent300,
        })
    }
}