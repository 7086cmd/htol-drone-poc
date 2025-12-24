# CLAUDE.md - AI Assistant Guide for HTOL Drone POC

## Project Overview

This is a **Horizontal Take-Off and Landing (HTOL) Drone** proof-of-concept written in embedded Rust. It's a learning project that implements a complete flight control system for a drone that combines fixed-wing and rotary-wing characteristics.

**Key Characteristics:**
- Bare-metal embedded Rust (`no_std`) running on STM32H743VGT6
- Async architecture using Embassy framework
- Real-time flight control with sensor fusion and PID controllers
- Personal learning project - expect iterative development and TBA components

## Quick Reference

### Essential Information

**Target Hardware:** STM32H743VGT6 (ARM Cortex-M7, 400 MHz)
**Rust Edition:** 2024
**Build Target:** `thumbv7em-none-eabi`
**Main Entry Point:** `drone/src/main.rs`
**Build & Flash:** `cargo run` (uses probe-rs runner)

### Core Dependencies

| Dependency | Purpose |
|------------|---------|
| `embassy-*` | Async embedded framework (executor, HAL, time, sync) |
| `ahrs` | Madgwick filter for sensor fusion |
| `nalgebra` | Linear algebra (quaternions, vectors) |
| `pid` | PID controller implementation |
| `nmea` | GPS sentence parsing |
| `dshot-frame` | Motor ESC communication protocol |
| `defmt` | Efficient logging for embedded systems |

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                  Flight Control System                       │
│                  (STM32H743VGT6 @ 400 MHz)                  │
├─────────────────────────────────────────────────────────────┤
│  Async Task Executor (Embassy)                              │
│                                                              │
│  ┌────────────────────┐     ┌─────────────────────────┐    │
│  │  Sensor Task       │     │  Control Task           │    │
│  │  @ 100 Hz          │────▶│  @ 100 Hz (TIM7 IRQ)   │    │
│  │                    │     │                         │    │
│  │  - I2C: IMU        │     │  - Outer Loop @ 50 Hz   │    │
│  │  - I2C: Mag        │     │    (Alt, Speed → Cmds)  │    │
│  │  - I2C: Baro       │     │  - Inner Loop @ 250 Hz  │    │
│  │  - UART: GPS       │     │    (Attitude Control)   │    │
│  │  - Madgwick AHRS   │     │  - Stall Detection      │    │
│  └────────────────────┘     └─────────────────────────┘    │
│           │                           │                     │
│           ▼                           ▼                     │
│     TELEMETRY (shared)          COMMAND (shared)            │
│     - Roll, Pitch, Yaw          - Motor throttle            │
│     - Altitude, Speed           - Servo angles              │
│     - Position, Velocity        - Steering                  │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │  Motion Controller (DShot + PWM)                   │    │
│  │  - 2x EDF Motors (DShot protocol)                  │    │
│  │  - 4x Servos (PWM @ 50 Hz)                         │    │
│  └────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Control Loop Architecture

**Dual-Loop Nested PID Design:**

1. **Outer Loop @ 50 Hz (Slower, Position/Velocity)**
   - Altitude PID → Pitch setpoint
   - Speed PID → Throttle command
   - Tuned for smooth response (Kp=0.3, Ki=0.01, Kd=0.5)

2. **Inner Loop @ 250 Hz (Faster, Attitude)**
   - Roll PID (Kp=0.8, fast response)
   - Pitch PID (Kp=0.7, fast response)
   - Yaw PID (Kp=0.5)
   - Higher bandwidth for disturbance rejection

**Implementation:** `drone/src/control/pid.rs`

### Data Flow

```
Sensors → I2C/UART → Raw Data → Madgwick AHRS → Telemetry (shared)
                                                      ↓
                                                 Control Task
                                                      ↓
                                              PID Controllers
                                                      ↓
                                                  Command (shared)
                                                      ↓
                                              Motion Controller
                                                      ↓
                                          DShot Frames + PWM Signals
                                                      ↓
                                              Motors + Servos
```

## File Structure Guide

### Core Files

```
drone/src/
├── main.rs              # Entry point, hardware init, task spawning
├── consts.rs            # Constants (Madgwick beta, sensor frequency)
├── types/
│   └── mod.rs           # Telemetry and Command data structures
├── control/
│   ├── mod.rs           # MotionController for PWM/DShot output
│   └── pid.rs           # PID controller implementation
└── sensor/
    ├── mod.rs           # Sensor module definitions
    ├── fusion.rs        # Madgwick AHRS sensor fusion
    ├── imu.rs           # MPU6050 IMU + HMC5883L Mag + BMP180 Baro
    ├── gps.rs           # NEO-6M GPS driver (NMEA parsing)
    └── pitot.rs         # MS4525DO airspeed sensor (optional)
```

### Key File Details

#### `drone/src/main.rs`
- **Purpose:** Entry point and system initialization
- **Key Functions:**
  - `main()` - STM32H743 clock setup (400 MHz), peripheral init
  - Task spawning: sensor_task, control_task, status LED
- **Important:** Uses `#![no_std]` and `#![no_main]` - bare-metal embedded
- **Clock Config:** System: 400 MHz, AHB: 200 MHz, APB1/2: 100 MHz

#### `drone/src/types/mod.rs`
- **Purpose:** Shared data structures for inter-task communication
- **Telemetry Struct:** Roll, pitch, yaw, altitude, speed, vertical speed, timestamp
- **Command Struct:** Yaw, roll, alt (pitch), speed (throttle), steer
- **Usage:** Shared via `embassy_sync::Mutex` between tasks

#### `drone/src/control/pid.rs`
- **Purpose:** Nested PID control loops
- **Key Logic:**
  - Outer loop: Altitude → Pitch, Speed → Throttle
  - Inner loop: Roll, Pitch, Yaw control
  - Stall detection: Speed < 15 m/s AND Pitch > 20° → boost throttle
- **Note:** Currently not fully integrated into main loop

#### `drone/src/sensor/fusion.rs`
- **Purpose:** Sensor fusion using Madgwick AHRS algorithm
- **Inputs:** Gyroscope, accelerometer, magnetometer (NED frame)
- **Outputs:** Quaternion → Euler angles (roll, pitch, yaw)
- **Parameters:** Beta = 0.33, Sample freq = 100 Hz

#### `drone/src/control/mod.rs`
- **Purpose:** Motor and servo control abstraction
- **MotionController Struct:**
  - Left/Right EDF motors (DShot protocol)
  - Aileron, elevator, rudder servos (PWM)
  - Undercarriage and nose wheel steering
- **DShot:** 0-2000 range for throttle
- **PWM Servos:** 1.0-2.0 ms pulse width, 50 Hz frequency

## Hardware Configuration

### Sensors (I2C)

| Sensor | Address | Purpose | Interface |
|--------|---------|---------|-----------|
| MPU6050 | 0x68 | IMU (Accel + Gyro) | I2C @ 400 kHz |
| HMC5883L | 0xE8 | Magnetometer | I2C @ 400 kHz |
| BMP180 | 0x77 | Barometer (altitude) | I2C @ 400 kHz |
| MS4525DO | 0x28 | Pitot (airspeed) | I2C @ 400 kHz |

### GPS (UART)

| Module | Baud Rate | Protocol | Purpose |
|--------|-----------|----------|---------|
| NEO-6M | 9600 | NMEA | Position, velocity, time |

### Motors & Servos

**Motors (DShot):**
- 2x 70mm EDF motors (CW/CCW pair)
- 80A BLHeli ESCs with DShot support
- Thrust: 1.8 kg each (3.6 kg total)

**Servos (PWM @ 50 Hz):**
- 2x MG90S (Ailerons)
- 1x MG90S (Elevator)
- 1x MG996R (Undercarriage steering)
- 1x MG955 (Nose wheel steering)

## Development Guidelines

### Code Style

1. **No Standard Library:** This is `no_std` embedded - no heap, limited stack
2. **Async First:** Use Embassy async/await for all I/O operations
3. **Defmt Logging:** Use `defmt::info!()`, `defmt::warn!()`, etc. (NOT println!)
4. **Error Handling:** Prefer `unwrap()` carefully or proper error propagation
5. **Fixed-Point Math:** Avoid floating-point in tight loops when possible
6. **NED Frame:** All position/velocity in North-East-Down coordinates

### Important Conventions

**Coordinate Systems:**
- **Body Frame:** Roll (X), Pitch (Y), Yaw (Z)
- **NED Frame:** North (X), East (Y), Down (Z)
- **Angles:** Degrees for Telemetry, radians internally for calculations

**Units:**
- Altitude: meters (AGL = Above Ground Level, MSL = Mean Sea Level)
- Speed: meters per second (m/s)
- Angular rates: degrees per second (°/s)
- Time: microseconds (µs) for timestamps

**Timing:**
- Sensor task: 10 ms period (100 Hz)
- Control task: 10 ms period (100 Hz) triggered by TIM7
- Outer loop: 20 ms period (50 Hz) - every 2nd control iteration
- Inner loop: 4 ms period (250 Hz) - 5x updates between outer loops

### Common Tasks

#### Building and Flashing

```bash
# Build for target
cargo build --release

# Flash to STM32H743
cargo run --release

# Build without flashing
cargo build --target thumbv7em-none-eabi
```

#### Debugging

```bash
# View RTT logs (defmt output)
# probe-rs automatically attaches when using `cargo run`

# Check build configuration
cat .cargo/config.toml

# Verify chip connection
probe-rs list
```

#### Adding a New Sensor

1. Create driver in `drone/src/sensor/new_sensor.rs`
2. Add I2C initialization in `main.rs`
3. Integrate into sensor_task (100 Hz loop)
4. Update Telemetry struct if new data fields needed
5. Update sensor fusion if affects attitude estimation

#### Modifying Control Logic

1. Edit `drone/src/control/pid.rs`
2. Tune PID constants (Kp, Ki, Kd)
3. Test outer loop (altitude/speed) separately from inner loop (attitude)
4. Update Command struct if new control outputs needed
5. Implement in MotionController for actuation

## Safety Features

### Current Implementations

1. **Stall Detection:** `drone/src/control/pid.rs`
   - Triggers when: Speed < 15 m/s AND Pitch > 20°
   - Action: Increase throttle to 1.8x current value

2. **Sensor Timeout:** 50 ms timeout for sensor reads
   - If exceeded: Enter safe mode (details TBA)

3. **Command Validation:** Bounds checking on control surface angles
   - Prevents servo damage from excessive commands

### Future Safety Features (Planned)

- Battery voltage monitoring
- Geofencing
- Return-to-home on signal loss
- Automatic landing on critical errors

## Build Configuration

### Memory Layout

- **Flash:** 2 MB (STM32H743VGT6)
- **RAM:** 1 MB
- **Linker Script:** Custom `link.x` with defmt support

### Optimization

```toml
[profile.dev]
opt-level = "z"        # Optimize for size
lto = true             # Link-time optimization

[profile.release]
opt-level = "z"
lto = true
debug = false
strip = true
```

### Target Configuration

```toml
[build]
target = "thumbv7em-none-eabi"

[target.thumbv7em-none-eabi]
runner = "probe-rs run --chip STM32H743VGTx"
```

## Troubleshooting

### Common Issues

**Issue:** `error: linking with 'rust-lld' failed`
- **Cause:** Memory overflow or incorrect linker script
- **Fix:** Check `link.x`, reduce binary size, verify memory layout

**Issue:** `Error: probe-rs: No probe found`
- **Cause:** ST-Link not connected or drivers missing
- **Fix:** Check USB connection, install ST-Link drivers, verify with `probe-rs list`

**Issue:** Sensor reads returning zeros
- **Cause:** I2C not initialized, wrong address, pull-ups missing
- **Fix:** Verify I2C initialization in `main.rs`, check sensor addresses, hardware pull-ups

**Issue:** Control loop not executing
- **Cause:** TIM7 interrupt not configured, task not spawned
- **Fix:** Check timer initialization, verify task spawn in `main.rs`

**Issue:** DShot motors not responding
- **Cause:** Wrong GPIO pins, DMA not configured, ESC not armed
- **Fix:** Verify pin mappings in `control/mod.rs`, check DMA setup, ESC calibration

## Testing Strategy

### Current Testing (Limited)

This project currently lacks automated tests due to embedded hardware dependencies.

### Manual Testing Approach

1. **Sensor Validation:** Read raw sensor values via defmt logs
2. **Fusion Validation:** Compare AHRS output with known orientations
3. **PID Tuning:** Bench test with servos disconnected, observe response
4. **Motor Test:** Gradual throttle increase, verify direction and thrust
5. **Integration:** Full system test in controlled environment

### Future Testing Plans

- Hardware-in-the-loop (HIL) simulation
- Unit tests for mathematical functions (quaternion ops, PID)
- Mock sensor data for sensor fusion validation
- Automated flight logs analysis

## Additional Resources

### Documentation

- **README.md:** Comprehensive project documentation (hardware specs, design rationale)
- **Cargo.toml:** Dependencies and build configuration
- **Embed.toml:** Embedded toolchain configuration

### External References

- [Embassy Framework](https://embassy.dev/) - Async embedded Rust
- [STM32H7 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [Madgwick AHRS Algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [DShot Protocol Specification](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)

### Helpful Commands

```bash
# Format code
cargo fmt

# Check without building
cargo check

# View dependencies
cargo tree

# Clean build artifacts
cargo clean

# Update dependencies
cargo update

# View device memory usage
cargo size --release -- -A
```

## Project Status

**Current State:** Active development, basic framework complete

**Implemented:**
- Hardware initialization and clock configuration
- Multi-sensor I2C communication
- GPS NMEA parsing
- Madgwick AHRS sensor fusion
- Dual-loop PID controller architecture (defined)
- DShot and PWM generation logic

**In Progress:**
- Integration of PID controllers into main control loop
- MotionController implementation and testing
- Comprehensive telemetry output

**Future Work:**
- Orange Pi Zero 3 companion computer integration
- Autonomous flight modes
- Advanced path planning
- Computer vision integration

## Contributing Notes

When making changes:

1. **Test on hardware:** Changes must be flashed and tested on actual STM32H743
2. **Log extensively:** Use defmt for debugging, especially in interrupt handlers
3. **Respect timing:** Sensor/control tasks run at 100 Hz - keep execution time < 10 ms
4. **Document constants:** Magic numbers should be in `consts.rs` with explanations
5. **Safety first:** Any control logic changes require thorough validation
6. **Commit messages:** Follow conventional commits (feat:, fix:, docs:, etc.)

## Contact & License

**License:** MIT (see LICENSE file)
**Author:** Ethan Wu
**Project Type:** Personal learning project / POC

---

*Last Updated: 2025-12-24*
*Generated for Claude Code AI Assistant*
