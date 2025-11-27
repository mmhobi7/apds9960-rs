use hal::blocking::i2c;
use {
    register::{Enable, GConfig1, GConfig4},
    Apds9960, BitFlags, Error, GestureDataThreshold, Register, DEV_ADDR,
};

/// Gesture engine configuration.
impl<I2C, E> Apds9960<I2C>
where
    I2C: i2c::Write<Error = E>,
{
    /// Enable gesture detection
    pub fn enable_gesture(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::GEN, true)
    }

    /// Disable gesture detection
    pub fn disable_gesture(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::GEN, false)
    }

    /// Enable gesture mode.
    ///
    /// This can be automatically enabled (depending on proximity thresholds)
    /// and disabled (see GMODE on datasheet).
    pub fn enable_gesture_mode(&mut self) -> Result<(), Error<E>> {
        self.set_flag_gconfig4(GConfig4::GMODE, true)
    }

    /// Disable gesture mode.
    ///
    /// This can be automatically enabled (depending on proximity thresholds)
    /// and disabled (see GMODE on datasheet).
    pub fn disable_gesture_mode(&mut self) -> Result<(), Error<E>> {
        self.set_flag_gconfig4(GConfig4::GMODE, false)
    }

    /// Enable gesture interrupt generation
    pub fn enable_gesture_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_flag_gconfig4(GConfig4::GIEN, true)
    }

    /// Disable gesture interrupt generation
    pub fn disable_gesture_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_flag_gconfig4(GConfig4::GIEN, false)
    }

    /// Set the threshold of amount of available data in the gesture FIFO registers.
    pub fn set_gesture_data_level_threshold(
        &mut self,
        threshold: GestureDataThreshold,
    ) -> Result<(), Error<E>> {
        use GestureDataThreshold as GDTH;
        let flags;
        match threshold {
            GDTH::Th1 => flags = (false, false),
            GDTH::Th4 => flags = (false, true),
            GDTH::Th8 => flags = (true, false),
            GDTH::Th16 => flags = (true, true),
        }
        let new = self
            .gconfig1
            .with(GConfig1::GFIFOTH1, flags.0)
            .with(GConfig1::GFIFOTH0, flags.1);
        self.config_register(&new)?;
        self.gconfig1 = new;
        Ok(())
    }

    /// Set the gesture proximity entry threshold.
    pub fn set_gesture_proximity_entry_threshold(&mut self, threshold: u8) -> Result<(), Error<E>> {
        self.write_register(Register::GPENTH, threshold)
    }

    /// Set the gesture proximity exit threshold.
    pub fn set_gesture_proximity_exit_threshold(&mut self, threshold: u8) -> Result<(), Error<E>> {
        self.write_register(Register::GPEXTH, threshold)
    }

    /// Set the gesture up offset.
    pub fn set_gesture_up_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
        self.write_register(Register::GOFFSET_U, offset as u8)
    }

    /// Set the gesture down offset.
    pub fn set_gesture_down_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
        self.write_register(Register::GOFFSET_D, offset as u8)
    }

    /// Set the gesture left offset.
    pub fn set_gesture_left_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
        self.write_register(Register::GOFFSET_L, offset as u8)
    }

    /// Set the gesture right offset.
    pub fn set_gesture_right_offset(&mut self, offset: i8) -> Result<(), Error<E>> {
        self.write_register(Register::GOFFSET_R, offset as u8)
    }

    /// Set the gesture up, down, left and right offsets.
    pub fn set_gesture_offsets(
        &mut self,
        offset_up: i8,
        offset_down: i8,
        offset_left: i8,
        offset_right: i8,
    ) -> Result<(), Error<E>> {
        self.i2c
            .write(
                DEV_ADDR,
                &[
                    Register::GOFFSET_U,
                    offset_up as u8,
                    offset_down as u8,
                    Register::GPULSE,  // Skip GPULSE register
                    offset_left as u8,
                    0,  // Skip register 0xA8
                    offset_right as u8,
                ],
            )
            .map_err(Error::I2C)
    }

    /// Enable all gesture photodiodes during gesture mode.
    pub fn enable_all_gesture_photodiodes(&mut self) -> Result<(), Error<E>> {
        self.write_register(Register::GCONF3, 0)
    }

    /// Set gesture photodiode dimensions.
    ///
    /// * `up_down`: Enable up/down photodiodes (true) or disable (false)
    /// * `left_right`: Enable left/right photodiodes (true) or disable (false)
    pub fn set_gesture_dimensions(&mut self, up_down: bool, left_right: bool) -> Result<(), Error<E>> {
        let mut mask = 0u8;
        if !up_down {
            mask |= 0b0000_0011; // Disable up and down
        }
        if !left_right {
            mask |= 0b0000_1100; // Disable left and right
        }
        self.write_register(Register::GCONF3, mask)
    }
}

impl<I2C, E> Apds9960<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
{
    /// Set gesture exit persistence.
    ///
    /// Number of consecutive gesture end occurrences to exit gesture mode.
    /// * `persistence`: 1, 2, 4, or 7 consecutive occurrences
    pub fn set_gesture_exit_persistence(&mut self, persistence: u8) -> Result<(), Error<E>> {
        let value = match persistence {
            1 => 0b00,
            2 => 0b01,
            4 => 0b10,
            7 => 0b11,
            _ => 0b01, // default to 2
        };
        let mut gconf1 = self.read_register(Register::GCONFIG1)?;
        gconf1 = (gconf1 & 0x9F) | (value << 5);
        self.write_register(Register::GCONFIG1, gconf1)
    }

    /// Set gesture exit mask.
    ///
    /// Determines which photodiode pairs are included in gesture exit comparison.
    /// * `mask`: Bitmask where bit 0=UD, 1=LR, 2=Both, 3=All
    pub fn set_gesture_exit_mask(&mut self, mask: u8) -> Result<(), Error<E>> {
        let mut gconf1 = self.read_register(Register::GCONFIG1)?;
        gconf1 = (gconf1 & 0xE7) | ((mask & 0x03) << 3);
        self.write_register(Register::GCONFIG1, gconf1)
    }

    /// Read gesture enter threshold.
    pub fn get_gesture_proximity_entry_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::GPENTH)
    }

    /// Read gesture exit threshold.
    pub fn get_gesture_proximity_exit_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::GPEXTH)
    }
}
