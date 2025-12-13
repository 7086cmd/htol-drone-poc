#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_time::TICK_HZ;
use embassy_time::{Duration, Timer};

mod sensor;
mod control;
mod consts;
mod types;

fn get_stm_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    #[cfg(debug_assertions)]
    let dbgmcu = embassy_stm32::pac::DBGMCU;
    #[cfg(debug_assertions)]
    dbgmcu.cr().modify(|w| {
        w.set_dbgsleep_d1(true);
        w.set_dbgstby_d1(true);
        w.set_dbgstop_d1(true);
    });
    {
        info!("The tick frequency is {} Hz", TICK_HZ);
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV4);
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(400),
            mode: HseMode::Oscillator,
        });
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Default::default()); // needed for RNG
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV1),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    config
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(get_stm_config());
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
