use hal::i2c;
use {Apds9960, Error};

/// Gesture direction codes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gesture {
    /// No gesture detected.
    None = 0,
    /// Swipe up.
    Up = 1,
    /// Swipe down.
    Down = 2,
    /// Swipe left.
    Left = 3,
    /// Swipe right.
    Right = 4,
}

impl<I2C, E> Apds9960<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Decode a gesture by reading the FIFO data and applying the same algorithm
    /// used in the Python/C++ drivers (filtering, ratios, deltas, then decision tree).
    pub fn decode_gesture(&mut self) -> nb::Result<Gesture, Error<E>> {
        if !self.is_gesture_data_valid().map_err(nb::Error::Other)? {
            return Err(nb::Error::WouldBlock);
        }

        let mut buffer = [0u8; 128];
        let mut datasets = [[0u8; 4]; 32];
        let mut dataset_count = 0;

        loop {
            if !self.is_gesture_data_valid().map_err(nb::Error::Other)? {
                break;
            }

            let level = self.read_gesture_data_level().map_err(nb::Error::Other)?;
            if level == 0 {
                break;
            }

            let byte_count = core::cmp::min(buffer.len(), 4 * level as usize);
            match self.read_gesture_data(&mut buffer[..byte_count]) {
                Ok(_) => {}
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(e)) => return Err(nb::Error::Other(e)),
            }

            for chunk in buffer[..byte_count].chunks_exact(4) {
                let (u, d, l, r) = (chunk[0], chunk[1], chunk[2], chunk[3]);
                if u >= 30
                    && d >= 30
                    && l >= 30
                    && r >= 30
                    && !(u == 0 && d == 0 && l == 0 && r == 0)
                    && !(u == 255 && d == 255 && l == 255 && r == 255)
                {
                    if dataset_count < datasets.len() {
                        datasets[dataset_count] = [u, d, l, r];
                        dataset_count += 1;
                    } else {
                        break;
                    }
                }
            }
        }

        if dataset_count < 2 {
            return Ok(Gesture::None);
        }

        let first = datasets[0];
        let last = datasets[dataset_count - 1];

        let f_r_ud =
            ((first[0] as i32 - first[1] as i32) * 100) / (first[0] as i32 + first[1] as i32);
        let f_r_lr =
            ((first[2] as i32 - first[3] as i32) * 100) / (first[2] as i32 + first[3] as i32);
        let l_r_ud = ((last[0] as i32 - last[1] as i32) * 100) / (last[0] as i32 + last[1] as i32);
        let l_r_lr = ((last[2] as i32 - last[3] as i32) * 100) / (last[2] as i32 + last[3] as i32);

        let delta_ud = l_r_ud - f_r_ud;
        let delta_lr = l_r_lr - f_r_lr;

        let state_ud = if delta_ud >= 30 {
            1
        } else if delta_ud <= -30 {
            -1
        } else {
            0
        };

        let state_lr = if delta_lr >= 30 {
            1
        } else if delta_lr <= -30 {
            -1
        } else {
            0
        };

        let gesture = match (state_ud, state_lr) {
            (-1, 0) => Gesture::Up,
            (1, 0) => Gesture::Down,
            (0, -1) => Gesture::Left,
            (0, 1) => Gesture::Right,
            (-1, 1) if delta_ud.abs() > delta_lr.abs() => Gesture::Up,
            (-1, 1) => Gesture::Right,
            (1, -1) if delta_ud.abs() > delta_lr.abs() => Gesture::Down,
            (1, -1) => Gesture::Left,
            (-1, -1) if delta_ud.abs() > delta_lr.abs() => Gesture::Up,
            (-1, -1) => Gesture::Left,
            (1, 1) if delta_ud.abs() > delta_lr.abs() => Gesture::Down,
            (1, 1) => Gesture::Right,
            _ => Gesture::None,
        };

        Ok(self.rotate_gesture(gesture))
    }

    /// Set the rotation offset that is applied to all decoded gestures.
    pub fn set_rotation(&mut self, degrees: u16) -> Result<(), Error<E>> {
        if ![0, 90, 180, 270].contains(&degrees) {
            return Err(Error::InvalidRotation);
        }
        self.rotation = degrees;
        Ok(())
    }

    /// Read the current rotation offset.
    pub fn rotation(&self) -> u16 {
        self.rotation
    }

    fn rotate_gesture(&self, gesture: Gesture) -> Gesture {
        if self.rotation == 0 {
            return gesture;
        }
        let dir_lookup = [Gesture::Up, Gesture::Right, Gesture::Down, Gesture::Left];
        let idx = dir_lookup.iter().position(|&g| g == gesture).unwrap_or(0);
        let rotated_idx = (idx + (self.rotation / 90) as usize) % 4;
        dir_lookup[rotated_idx]
    }
}
