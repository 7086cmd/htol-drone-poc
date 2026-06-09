//! Lidar sensor interface
//! Uses UART communication to interact with the Lidar sensor.
//! 
//! Used LiDAR: TFmini-S LiDAR Module (https://www.sparkfun.com/tfmini-s-micro-lidar-module.html).
//! 
//! 
use embassy_stm32::{usart::Uart, mode::Async};

pub enum LidarError {
    UartError,
    ParseError,
}

pub struct Lidar<'a> {
    pub uart: Uart<'a, Async>,
}

impl<'a> Lidar<'a> {
    pub fn new(uart: Uart<'a, Async>) -> Result<Self, LidarError> {
        uart.set_baudrate(115200).map_err(|_| LidarError::UartError)?;
        Ok(Self { uart })
    }

    pub async fn read(&mut self) -> Result<(u16, u16, u16), LidarError> {
        let mut buffer = [0u8; 9];
        self.uart
            .read_exact(&mut buffer)
            .await
            .map_err(|_| LidarError::UartError)?;

        // Validate frame header
        if buffer[0] != 0x59 || buffer[1] != 0x59 {
            return Err(LidarError::ParseError);
        }
        
        let dist_l = buffer[2] as u16;
        let dist_h = buffer[3] as u16;
        let distance = (dist_h << 8) | dist_l;

        let strength_l = buffer[4] as u16;
        let strength_h = buffer[5] as u16;
        let strength = (strength_h << 8) | strength_l;

        let temp_l = buffer[6] as u16;
        let temp_h = buffer[7] as u16;
        let temperature = ((temp_h << 8) | temp_l) / 8 - 256;

        let checksum = buffer[8] as u16;
        // Checksum is the lower 8 bits of the cumulative sum of the numbers of the first 8 bytes. 
        let sum: u16 = buffer[..8].iter().map(|&b| b as u16).sum();
        if (sum & 0xFF) == checksum {
            Ok((distance, strength, temperature))
        } else {
            Err(LidarError::ParseError)
        }
    }
}