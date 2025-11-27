use hal::i2c;
use {
    register::{Config1, Enable},
    Apds9960, BitFlags, Error, Register, DEV_ADDR,
};

macro_rules! impl_set_flag_reg {
    ($method:ident, $reg:ident) => {
        pub(crate) fn $method(&mut self, flag: u8, value: bool) -> Result<(), Error<E>> {
            let new = self.$reg.with(flag, value);
            self.config_register(&new)?;
            self.$reg = new;
            Ok(())
        }
    };
}

/// Common configuration.
impl<I2C, E> Apds9960<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Initialize the sensor with sensible defaults.
    ///
    /// This sets up the sensor with defaults similar to the Python and C++ drivers:
    /// - All engines disabled initially
    /// - Proximity: 4x gain, 100mA LED, threshold 0-50, 4 cycles persistence
    /// - Light: 4x gain, 256 cycles integration time (712ms)
    /// - Gesture: 4x gain, 100mA LED, entry thresh 40, exit thresh 30
    ///
    /// After calling this, you should enable the specific engines you need.
    ///
    /// This function first probes the I2C bus to verify the device is present
    /// before attempting configuration.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Probe the device to verify it's present before initialization
        self.probe()?;
        
        // Disable all features first
        self.disable()?;

        // Set proximity defaults
        self.write_register(Register::PPULSE, 0x87)?; // 16us, 8 pulses
        self.write_register(Register::POFFSET_UR, 0)?;
        self.write_register(Register::POFFSET_DL, 0)?;
        self.write_register(Register::PILT, 0)?;
        self.write_register(Register::PIHT, 50)?;

        // Set light defaults
        self.write_register(Register::ATIME, 0)?; // 256 cycles = 712ms
        self.write_double_register(Register::AILTL, 0xFFFF)?;
        self.write_double_register(Register::AIHTL, 0)?;

        // Set persistence: 4 proximity cycles, 0 ALS cycles
        self.write_register(Register::PERS, 0x40)?; // (4 << 4) | 0

        // Set wait time
        self.write_register(Register::WTIME, 246)?; // 27ms

        // CONFIG1: no 12x wait
        self.write_register(Register::CONFIG1, 0x60)?;

        // CONFIG2: LED boost 100%, no saturation interrupts
        self.write_register(Register::CONFIG2, 0x01)?;

        // CONFIG3: all photodiodes enabled
        self.write_register(Register::CONFIG3, 0)?;

        // Set gesture defaults
        self.write_register(Register::GPENTH, 40)?; // Entry threshold
        self.write_register(Register::GEXTH, 30)?; // Exit threshold
        self.write_register(Register::GCONF1, 0x40)?; // 4 events for int, 1 for exit
        self.write_register(Register::GCONF2, 0x41)?; // 4x gain, 100mA, 2.8ms wait
        self.write_register(Register::GPULSE, 0xC9)?; // 32us, 10 pulses
        self.write_register(Register::GCONF3, 0)?; // All photodiodes active
        self.write_register(Register::GCONFIG4, 0)?; // Gesture interrupts disabled initially

        // Set gesture offsets to 0
        self.write_register(Register::GOFFSET_U, 0)?;
        self.write_register(Register::GOFFSET_D, 0)?;
        self.write_register(Register::GOFFSET_L, 0)?;
        self.write_register(Register::GOFFSET_R, 0)?;

        // Set control register: 100mA LED, 4x proximity gain, 4x ALS gain
        self.write_register(Register::CONTROL, 0x09)?; // (0 << 6) | (2 << 2) | 1

        // Enable power
        self.enable()
    }

    /// Turn power on.
    pub fn enable(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::PON, true)
    }

    /// Enable only this engine (like color/proximity) without touching others.
    ///
    /// Already used by `enable_proximity_sensor`.
    pub fn enable_power(&mut self) -> Result<(), Error<E>> {
        self.enable()
    }

    /// Disable only power.
    pub fn disable_power(&mut self) -> Result<(), Error<E>> {
        self.disable()
    }

    /// Set the current sensor mode bits.
    pub fn set_mode(&mut self, mode: u8) -> Result<(), Error<E>> {
        let mut enable = self.read_register(Register::ENABLE)?;
        enable &= !Enable::ALL;
        enable |= mode;
        self.write_register(Register::ENABLE, enable)?;
        Ok(())
    }

    /// Read the current enable register (mode bits).
    pub fn get_mode(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::ENABLE)
    }

    /// Deactivate everything and put the device to sleep.
    pub fn disable(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::ALL, false)
    }

    /// Enable the wait feature.
    ///
    /// Enables delay between proximity and / or color and ambient light cycles.
    /// The duration of the wait can be configured with
    /// [`set_wait_time()`](struct.Apds9960.html#method.set_wait_time) and
    /// [`enable_wait_long()`](struct.Apds9960.html#method.enable_wait_long).
    pub fn enable_wait(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::WEN, true)
    }

    /// Disable the wait feature.
    pub fn disable_wait(&mut self) -> Result<(), Error<E>> {
        self.set_flag_enable(Enable::WEN, false)
    }

    /// Enable long wait.
    ///
    /// The wait time will be multiplied by 12 so that each cycle takes 0.03s.
    /// See also: [`set_wait_time()`](struct.Apds9960.html#method.set_wait_time).
    ///
    /// Wait must be enabled with [`enable_wait()`](struct.Apds9960.html#method.enable_wait).
    pub fn enable_wait_long(&mut self) -> Result<(), Error<E>> {
        self.set_flag_config1(Config1::WLONG, true)
    }

    /// Disable long wait.
    pub fn disable_wait_long(&mut self) -> Result<(), Error<E>> {
        self.set_flag_config1(Config1::WLONG, false)
    }

    /// Set the waiting time between proximity and / or color and ambient light cycles.
    ///
    /// The value parameter must be a 2's complement of the number of cycles.
    ///
    /// Per default this is set to `0xFF` (1 cycle) and each cycle has a fixed duration of 2.78ms
    /// except if long wait is enabled, then this time is multiplied by 12.
    ///
    /// This must be set before enabling proximity and / or color and ambient light detection.
    ///
    /// Waiting must be enabled with [`enable_wait()`](struct.Apds9960.html#method.enable_wait).
    /// Long wait can be enabled with [`enable_wait_long()`](struct.Apds9960.html#method.enable_wait_long).
    pub fn set_wait_time(&mut self, value: u8) -> Result<(), Error<E>> {
        self.write_register(Register::WTIME, value)
    }

    /// Force an interrupt.
    pub fn force_interrupt(&mut self) -> Result<(), Error<E>> {
        self.touch_register(Register::IFORCE)
    }

    /// Clear all *non-gesture* interrupts.
    pub fn clear_interrupts(&mut self) -> Result<(), Error<E>> {
        self.touch_register(Register::AICLEAR)
    }

    impl_set_flag_reg!(set_flag_enable, enable);
    impl_set_flag_reg!(set_flag_config1, config1);
    impl_set_flag_reg!(set_flag_config2, config2);
    impl_set_flag_reg!(set_flag_config3, config3);
    impl_set_flag_reg!(set_flag_gconfig4, gconfig4);

    pub(crate) fn config_register<T: BitFlags>(&mut self, reg: &T) -> Result<(), Error<E>> {
        self.write_register(T::ADDRESS, reg.value())
    }

    pub(crate) fn write_register(&mut self, address: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(DEV_ADDR, &[address, value])
            .map_err(Error::I2C)
    }

    pub(crate) fn write_double_register(
        &mut self,
        start_register: u8,
        value: u16,
    ) -> Result<(), Error<E>> {
        self.i2c
            .write(DEV_ADDR, &[start_register, value as u8, (value >> 8) as u8])
            .map_err(Error::I2C)
    }

    pub(crate) fn touch_register(&mut self, address: u8) -> Result<(), Error<E>> {
        self.i2c.write(DEV_ADDR, &[address]).map_err(Error::I2C)
    }
}
