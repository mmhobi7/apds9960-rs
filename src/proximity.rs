use hal::i2c;
use {
    register::{Config2, Config3, Enable, Pers, Status},
    Apds9960, BitFlags, Error, Register,
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
        self.set_proximity_up_right_offset(offset_up_right)?;
        self.set_proximity_down_left_offset(offset_down_left)
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
}
