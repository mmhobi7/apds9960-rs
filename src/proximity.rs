use hal::i2c;
use {
    register::{Config2, Config3, Control, Enable, Pers, Status},
    Apds9960, BitFlags, Error, Register, DEV_ADDR,
};

/// Proximity sensor implementation with comprehensive register access.
impl<I2C, E> Apds9960<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Enable proximity detection
    pub fn enable_proximity(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::PEN, true)
    }

    /// Disable proximity detection
    pub fn disable_proximity(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::PEN, false)
    }

    /// Enable proximity sensor with interrupts
    pub fn enable_proximity_sensor(&mut self, interrupts: bool) -> Result<(), Error<E>> {
        use crate::gains::ProximityGain;
        self.set_proximity_gain(ProximityGain::X4)?; // Default 4x gain
        if interrupts {
            self.enable_proximity_interrupts()?;
        } else {
            self.disable_proximity_interrupts()?;
        }
        self.enable()?;
        self.enable_proximity()
    }

    /// Disable proximity sensor
    pub fn disable_proximity_sensor(&mut self) -> Result<(), Error<E>> {
        self.disable_proximity_interrupts()?;
        self.disable_proximity()
    }

    /// Enable proximity interrupt generation
    pub fn enable_proximity_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::PIEN, true)
    }

    /// Disable proximity interrupt generation
    pub fn disable_proximity_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::PIEN, false)
    }

    /// Enable proximity saturation interrupt generation
    pub fn enable_proximity_saturation_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_flag_config2(Config2::PSIEN, true)
    }

    /// Disable proximity saturation interrupt generation
    pub fn disable_proximity_saturation_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_flag_config2(Config2::PSIEN, false)
    }

    /// Set the proximity interrupt low threshold.
    pub fn set_proximity_low_threshold(&mut self, threshold: u8) -> Result<(), Error<E>> {
        self.write_register(Register::PILT, threshold)
    }

    /// Get the proximity interrupt low threshold.
    pub fn get_proximity_low_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::PILT)
    }

    /// Set the proximity interrupt high threshold.
    pub fn set_proximity_high_threshold(&mut self, threshold: u8) -> Result<(), Error<E>> {
        self.write_register(Register::PIHT, threshold)
    }

    /// Get the proximity interrupt high threshold.
    pub fn get_proximity_high_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::PIHT)
    }

    /// Set the proximity up/right photodiode offset.
    pub fn set_proximity_up_right_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
        self.write_register(Register::POFFSET_UR, offset as u8)
    }

    /// Set the proximity down/left photodiode offset.
    pub fn set_proximity_down_left_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
        self.write_register(Register::POFFSET_DL, offset as u8)
    }

    /// Set the proximity up/right and down/left photodiode offset.
    pub fn set_proximity_offsets(
        &mut self,
        offset_up_right: i8,
        offset_down_left: i8,
    ) -> Result<(), Error<E>> {
        self.i2c
            .write(
                DEV_ADDR,
                &[
                    Register::POFFSET_UR,
                    offset_up_right as u8,
                    offset_down_left as u8,
                ],
            )
            .map_err(Error::I2C)
    }

    /// Set proximity interrupt persistence.
    ///
    /// This value controls how many consecutive out-of-threshold measurements
    /// are required before triggering a proximity interrupt.
    ///
    /// * `cycles`: Number of consecutive cycles (0-15)
    pub fn set_proximity_interrupt_persistence(&mut self, cycles: u8) -> Result<(), Error<E>> {
        let mut pers = self.read_register(Register::PERS)?;
        pers = (pers & !Pers::PPERS_MASK) | ((cycles & 0x0F) << Pers::PPERS_SHIFT);
        self.write_register(Register::PERS, pers)
    }

    /// Get proximity interrupt persistence.
    pub fn get_proximity_interrupt_persistence(&mut self) -> Result<u8, Error<E>> {
        let pers = self.read_register(Register::PERS)?;
        Ok((pers & Pers::PPERS_MASK) >> Pers::PPERS_SHIFT)
    }

    /// Enable proximity gain compensation.
    pub fn enable_proximity_gain_compensation(&mut self) -> Result<(), Error<E>> {
        self.set_flag_config3(Config3::PCMP, true)
    }

    /// Disable proximity gain compensation.
    pub fn disable_proximity_gain_compensation(&mut self) -> Result<(), Error<E>> {
        self.set_flag_config3(Config3::PCMP, false)
    }

    /// Get proximity gain compensation status.
    pub fn get_proximity_gain_compensation(&mut self) -> Result<u8, Error<E>> {
        let config3 = self.read_register(Register::CONFIG3)?;
        Ok((config3 >> 5) & 0x01)
    }

    /// Set proximity photodiode mask.
    ///
    /// Each bit masks a photodiode: bit 0=Right, 1=Left, 2=Down, 3=Up
    /// Setting a bit to 1 disables that photodiode.
    pub fn set_proximity_photodiode_mask(&mut self, mask: u8) -> Result<(), Error<E>> {
        let mut config3 = self.read_register(Register::CONFIG3)?;
        config3 = (config3 & 0xF0) | (mask & 0x0F);
        self.write_register(Register::CONFIG3, config3)
    }

    /// Get proximity photodiode mask.
    pub fn get_proximity_photodiode_mask(&mut self) -> Result<u8, Error<E>> {
        let config3 = self.read_register(Register::CONFIG3)?;
        Ok(config3 & 0x0F)
    }

    /// Clear proximity interrupt.
    pub fn clear_proximity_interrupt(&mut self) -> Result<(), Error<E>> {
        self.touch_register(Register::PICLEAR)
    }

    /// Read the proximity sensor data.
    ///
    /// Returns `nb::Error::WouldBlock` as long as the data is not ready.
    pub fn read_proximity(&mut self) -> nb::Result<u8, Error<E>> {
        if !self.is_proximity_data_valid().map_err(nb::Error::Other)? {
            return Err(nb::Error::WouldBlock);
        }
        self.read_register(Register::PDATA)
            .map_err(nb::Error::Other)
    }

    /// Read whether the proximity sensor data is valid.
    ///
    /// This is checked internally in `read_proximity()` as well.
    #[allow(clippy::wrong_self_convention)]
    pub fn is_proximity_data_valid(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_register(Register::STATUS)?;
        Ok(Status::create(status).is(Status::PVALID, true))
    }

    /// Set proximity gain.
    ///
    /// Value  Gain
    ///   0       1x
    ///   1       2x
    ///   2       4x
    ///   3       8x
    pub fn set_proximity_gain(&mut self, gain: u8) -> Result<(), Error<E>> {
        let mut control = self.read_register(Register::CONTROL)?;
        control &= 0b11110011;
        control |= (gain & 0x03) << 2;
        self.write_register(Register::CONTROL, control)
    }

    /// Get proximity gain.
    pub fn get_proximity_gain(&mut self) -> Result<u8, Error<E>> {
        let control = self.read_register(Register::CONTROL)?;
        Ok((control >> 2) & 0x03)
    }

    /// Set LED drive strength for proximity and ALS.
    ///
    /// Value    LED Current
    ///   0        100 mA
    ///   1         50 mA
    ///   2         25 mA
    ///   3         12.5 mA
    pub fn set_led_drive(&mut self, drive: u8) -> Result<(), Error<E>> {
        let mut control = self.read_register(Register::CONTROL)?;
        control &= 0b00111111;
        control |= (drive & 0x03) << 6;
        self.write_register(Register::CONTROL, control)
    }

    /// Get LED drive strength.
    pub fn get_led_drive(&mut self) -> Result<u8, Error<E>> {
        let control = self.read_register(Register::CONTROL)?;
        Ok((control >> 6) & 0x03)
    }

    /// Set LED boost current.
    ///
    /// Value  Boost Current
    ///   0        100%
    ///   1        150%
    ///   2        200%
    ///   3        300%
    pub fn set_led_boost(&mut self, boost: u8) -> Result<(), Error<E>> {
        let mut config2 = self.read_register(Register::CONFIG2)?;
        config2 &= 0b11001111;
        config2 |= (boost & 0x03) << 4;
        self.write_register(Register::CONFIG2, config2)
    }

    /// Get LED boost current.
    pub fn get_led_boost(&mut self) -> Result<u8, Error<E>> {
        let config2 = self.read_register(Register::CONFIG2)?;
        Ok((config2 >> 4) & 0x03)
    }

    /// Set proximity pulse count and length.
    ///
    /// * `count`: Number of pulses (0-255, typically 8)
    /// * `length`: Pulse length (0=4μs, 1=8μs, 2=16μs, 3=32μs)
    pub fn set_proximity_pulse(&mut self, count: u8, length: u8) -> Result<(), Error<E>> {
        let value = ((length & 0x03) << 6) | (count & 0x3F);
        self.write_register(Register::PPULSE, value)
    }

    /// Get proximity pulse settings.
    pub fn get_proximity_pulse(&mut self) -> Result<(u8, u8), Error<E>> {
        let value = self.read_register(Register::PPULSE)?;
        let count = value & 0x3F;
        let length = (value >> 6) & 0x03;
        Ok((count, length))
    }
}
