# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an experimental Horizontal Take-Off and Landing (HTOL) drone proof-of-concept written in Rust for embedded STM32H743VG microcontroller. The project uses Embassy async framework for embedded development and implements flight control systems, sensor fusion, and motor control for a fixed-wing drone that can take off and land horizontally like an aircraft.

## Build and Development Commands

### Building
```bash
# Build the firmware for STM32H743VG target
cargo build --target thumbv7em-none-eabi

# Build in release mode (optimized for embedded deployment)
cargo build --target thumbv7em-none-eabi --release

# Check code without building
cargo check --target thumbv7em-none-eabi
```

### Development Tools
```bash
# Format code
cargo fmt

# Run clippy for linting
cargo clippy --target thumbv7em-none-eabi

# Clean build artifacts
cargo clean
```

### Flashing and Debugging
The project uses probe-rs/defmt for debugging. Specific flash commands depend on your debug probe setup.

## Architecture Overview

### Hardware Target
- **Microcontroller**: STM32H743VG (Cortex-M7, 400MHz)
- **Flight Control**: Embassy async executor with real-time constraints
- **Sensors**: IMU (MPU6050), GPS (NEO-6M), Barometer, Pitot tube
- **Motors**: Dual EDF (Electric Ducted Fan) with DShot protocol
- **Servos**: Standard PWM servos for control surfaces (ailerons, elevator, rudder, steering)

### Software Architecture

#### Core Structure
- `src/main.rs` - Entry point with Embassy executor and system initialization
- `src/sensor/` - Sensor drivers and data fusion
  - `imu.rs` - Inertial Measurement Unit interface
  - `gps.rs` - GPS module handling
  - `fusion.rs` - Sensor data fusion algorithms
  - `pitot.rs` - Airspeed measurement
- `src/control/` - Flight control systems
  - `pid.rs` - PID controller implementations
  - `mod.rs` - Motion controller with EDF and servo control
- `src/types/` - Common data structures (Telemetry, Command)
- `src/consts.rs` - System constants

#### Key Dependencies
- **Embassy**: Async runtime (`embassy-executor`, `embassy-stm32`, `embassy-time`)
- **AHRS**: Attitude and Heading Reference System for sensor fusion
- **DShot**: Motor control protocol via `dshot-frame` crate
- **PID**: Control loop implementation
- **NMEA**: GPS data parsing
- **defmt**: Embedded logging framework

#### Hardware Abstraction
The `MotionController` struct manages all actuators:
- **EDF Control**: DShot protocol for precise motor speed control
- **Servo Control**: PWM signals for control surfaces (ailerons, elevator, rudder)
- **Steering**: Retractable gear control for ground operations

### Communication Protocols
- **I2C**: IMU and barometer communication
- **UART**: GPS module interface
- **PWM**: Servo control (1-2ms pulse width, 50Hz frequency)
- **DShot**: Bidirectional digital motor control

## Development Considerations

### Real-time Constraints
This is safety-critical flight control firmware. Maintain deterministic timing for:
- Sensor reading loops
- Control algorithm execution
- Motor/servo command transmission

### Memory Management
- `#![no_std]` environment - no heap allocation
- Use stack-allocated data structures
- Embassy provides async without heap allocation

### Safety Patterns
- All motor/servo functions return `Result<(), ControlError>`
- Input validation for servo angles (limited ranges)
- Fail-safe behavior on communication errors
- Sensor fusion for redundancy

### Testing Strategy
Embedded testing is limited. Focus on:
- Unit tests for control algorithms (when possible in std environment)
- Hardware-in-the-loop testing for integrated systems
- Ground testing before any flight attempts

## Key Constants and Configurations

### Clock Configuration
- System clock: 400MHz (PLL1_P)
- AHB clock: 200MHz
- APB clocks: 100MHz
- Embassy tick rate: 480MHz

### Control Surface Limits
- Aileron: ±90° range
- Elevator: ±30° range  
- Rudder: ±30° range
- Servo PWM: 1-2ms pulse width at 50Hz

### Motor Control
- DShot protocol range: 0-2000 (throttle commands)
- Dual EDF configuration with left/right channels
- Telemetry request capability via DShot