use embassy_stm32::{mode::Async, usart::Uart};
use nmea::sentences::rmc;

pub enum GpsError {
    UartError,
    ParseError,
}

pub struct NeoGps<'a> {
    pub uart: Uart<'a, Async>,
}

pub struct GpsData {
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub heading: Option<f32>,
    pub altitude_msl: Option<f32>,
    pub groundspeed: Option<f32>,
    pub num_satellites: Option<u32>,
    pub fix_quality: Option<u8>,
    pub format: Option<u8>, // GPS fix format (0 = invalid, 1 = GPS fix, 2 = DGPS fix)
}

impl<'a> NeoGps<'a> {
    pub fn new(uart: Uart<'a, Async>) -> Result<Self, GpsError> {
        uart.set_baudrate(9600).map_err(|_| GpsError::UartError)?;
        Ok(Self { uart })
    }

    pub async fn read(&mut self) -> Result<Option<GpsData>, GpsError> {
        let buffer = &mut [0u8; 128];
        self.uart
            .read_until_idle(buffer)
            .await
            .map_err(|_| GpsError::UartError)?;

        let result = nmea::parse_bytes(buffer).map_err(|_| GpsError::ParseError)?;

        match result {
            nmea::ParseResult::RMC(rmc) => Ok(Some(GpsData {
                latitude: rmc.lat.map(|lat| lat.to_degrees()),
                longitude: rmc.lon.map(|lon| lon.to_degrees()),
                heading: rmc.true_course.map(|hdg| hdg as f32),
                altitude_msl: None, // RMC does not provide altitude
                num_satellites: None, // RMC does not provide satellite count
                fix_quality: None, // RMC does not provide fix quality
                groundspeed: rmc.speed_over_ground.map(|spd| spd as f32),
                format: Some(match rmc.status {
                    nmea::sentences::FixStatus::Valid => 1,
                    nmea::sentences::FixStatus::Invalid => 0,
                }),
            })),
            nmea::ParseResult::GGA(gga) => Ok(Some(GpsData {
                latitude: gga.lat.map(|lat| lat.to_degrees()),
                longitude: gga.lon.map(|lon| lon.to_degrees()),
                heading: None, // GGA does not provide heading
                altitude_msl: gga.altitude.map(|alt| alt as f32),
                num_satellites: gga.num_of_satellites.map(|num| num as u32),
                fix_quality: gga.fix_quality.map(|fq| fq as u8),
                groundspeed: None, // GGA does not provide groundspeed
                format: gga.fix_quality.map(|fq| fq as u8),
            })),
            nmea::ParseResult::Unsupported(_) => Err(GpsError::ParseError),
            _ => Ok(None)
        }
    }
}
