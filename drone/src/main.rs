#![no_std]
#![no_main]
#![allow(dead_code)]

use {defmt_rtt as _, panic_probe as _};

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::{Channel, low_level::Timer as HalTimer, simple_pwm::SimplePwm};
use embassy_stm32::usart::Uart;
use embassy_stm32::{bind_interrupts, interrupt, pac};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::TICK_HZ;
use embassy_time::{Duration, Instant, Timer};

mod consts;
mod control;
mod sensor;
mod types;

use consts::*;
use control::{MotionController, pid::PidController};
use sensor::fusion::SensorFusion;
use types::{Command, Telemetry};

static TELEMETRY: Mutex<ThreadModeRawMutex, Option<Telemetry>> = Mutex::new(None);
static CONTROL_SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();
static PID_CONTROLLER: Mutex<ThreadModeRawMutex, Option<PidController>> = Mutex::new(None);

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

#[embassy_executor::task]
async fn sensor_task() {
    let mut sensor_fusion = SensorFusion::new(0.001, 0.33);
    let mut last_update = Instant::now();

    info!("Sensor task started");

    loop {
        if let Ok(telemetry) = read_sensors(&mut sensor_fusion).await {
            // Update shared telemetry
            {
                let mut telem = TELEMETRY.lock().await;
                *telem = Some(telemetry);
            }
            last_update = Instant::now();
        } else {
            error!("Sensor read failed");
        }

        Timer::after(Duration::from_millis(10)).await; // 100Hz sensor reading
    }
}

#[embassy_executor::task]
async fn control_task() {
    let mut pid_controller = PidController::new();
    let mut stall_detected = false;
    let mut last_sensor_time = Instant::now();
    let sensor_timeout = Duration::from_millis(50);

    info!("Control task started");

    loop {
        // Wait for control signal from TIM7
        CONTROL_SIGNAL.wait().await;

        let current_time = Instant::now();

        // Get latest telemetry
        let telemetry = {
            let telem = TELEMETRY.lock().await;
            telem.clone()
        };

        if let Some(telemetry) = telemetry {
            last_sensor_time = current_time;

            // Stall detection
            stall_detected = telemetry.speed < 15.0 && telemetry.pitch > 20.0;
            if stall_detected {
                warn!(
                    "STALL DETECTED: Speed={}, Pitch={}",
                    telemetry.speed, telemetry.pitch
                );
                pid_controller.speed.setpoint = 30.0; // Target higher speed
                pid_controller.roll.setpoint = 0.0; // Level wings
            }

            // Control calculations
            let command = pid_controller.update(&telemetry);

            // Apply controls (placeholder - needs actual MotionController instance)
            info!(
                "Controls: P={}, R={}, Y={}, S={}",
                command.alt, command.roll, command.yaw, command.speed
            );
        } else {
            // Sensor timeout check
            if current_time.duration_since(last_sensor_time) > sensor_timeout {
                error!("Sensor timeout - entering safe mode");
                pid_controller.reset();
            }
        }
    }
}

#[interrupt]
fn TIM7() {
    embassy_stm32::pac::TIM7.sr().write(|w| w.set_uif(false));

    CONTROL_SIGNAL.signal(());
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(get_stm_config());
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);

    // Setup TIM7 for 100Hz control loop trigger
    let mut timer = embassy_stm32::timer::low_level::Timer::new(p.TIM7);

    timer.set_tick_freq(Hertz::hz(100));

    timer.start();

    unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::TIM7) };

    info!("Flight control system initialized");

    // Spawn tasks
    spawner.spawn(sensor_task()).unwrap();
    spawner.spawn(control_task()).unwrap();

    // Status LED task
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

async fn read_sensors(fusion: &mut SensorFusion) -> Result<Telemetry, ()> {
    use num_traits::float::Float;
    
    let velocity = fusion.velocity;

    let speed = (velocity.x.powi(2) + velocity.y.powi(2)).sqrt();
    let alt = fusion.altitude_agl;
    let (roll, pitch, yaw) = fusion.get_euler();
    let vz = velocity.z;

    Ok(Telemetry {
        roll: roll.to_degrees(),
        pitch: pitch.to_degrees(),
        yaw: yaw.to_degrees(),
        alt,
        speed,
        speed_z: vz,
        timestamp_us: embassy_time::Instant::now().as_micros() as u64,
    })
}
