use embassy_stm32::{mode::Async, usart::Uart};

pub enum GpsError {
    UartError,
    ParseError,
}

pub struct NeoGps<'a> {
    pub uart: Uart<'a, Async>
}

pub struct GpsData {
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub altitude_msl: Option<f32>,
    pub num_satellites: Option<u32>,
    pub fix_quality: Option<u8>,
    pub format: Option<u8>, // GPS fix format (0 = invalid, 1 = GPS fix, 2 = DGPS fix)
}

impl<'a> NeoGps<'a> {
    pub fn new(uart: Uart<'a, Async>) -> Result<Self, GpsError> {
        uart.set_baudrate(9600).map_err(|_| GpsError::UartError)?;
        Ok(Self { uart })
    }

    pub async fn read(&mut self) -> Result<GpsData, GpsError> {
        let buffer = &mut [0u8; 128];
        self.uart.read_until_idle(buffer).await.map_err(|_| GpsError::UartError)?;

        let result = nmea::parse_bytes(buffer)
            .map_err(|_| GpsError::ParseError)?;

        match result {
            nmea::ParseResult::GGA(sentence) => Ok(GpsData {
                latitude: sentence.latitude.map(|lat| lat.to_degrees()),
                longitude: sentence.longitude.map(|lon| lon.to_degrees()),
                altitude_msl: sentence.altitude,
                num_satellites: sentence.fix_satellites,
                fix_quality: Some(0),
                format: None,
            }),
            _ => Err(GpsError::ParseError),
        }
    }
}
