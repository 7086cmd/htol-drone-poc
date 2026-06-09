mod taxi;
mod takeoff;
mod cruise;
mod landing;

pub enum FlightStage {
    Taxi(taxi::TaxiState), // TXI
    Takeoff(takeoff::TakeoffState), // TOF
    Cruise(cruise::CruiseState), // CRZ
    Landing(landing::LandingState), // LDG
}