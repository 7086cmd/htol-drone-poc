//! Flight control data types for telemetry and command structures.
//!
//! This module defines the core data structures used for inter-task communication
//! in the HTOL drone flight control system. These types are shared between the
//! sensor fusion task and the PID control task via `embassy_sync::Mutex`.
//!
//! # Coordinate Systems
//!
//! All telemetry uses **NED (North-East-Down)** coordinate frame conventions:
//! - **North**: Positive X-axis (forward)
//! - **East**: Positive Y-axis (right)
//! - **Down**: Positive Z-axis (downward, toward ground)
//!
//! # Control Flow
//!
//! ```text
//! Sensor Task (100 Hz)                Control Task (100 Hz)
//!        │                                    │
//!        ├──► Telemetry (shared) ────────────►│
//!        │                                    │
//!        │                                    ├──► PID Controllers
//!        │                                    │
//!        │◄──── Command (shared) ◄────────────┤
//!        │                                    │
//!        ▼                                    ▼
//! Motion Controller                    Next Control Cycle
//! ```
//!
//! # Thread Safety
//!
//! Both `Telemetry` and `Command` are `Copy` types to enable efficient,
//! lock-free reading from shared memory. Updates are atomic at the word level
//! on ARM Cortex-M7 (32-bit aligned f32 fields).

#![allow(dead_code)]

/// Real-time telemetry data from sensor fusion and state estimation.
///
/// This structure contains the complete flight state of the drone, updated
/// by the sensor task at 100 Hz and consumed by the control task for PID
/// calculations.
///
/// # Update Rate
///
/// - **Sensor Task**: 100 Hz (10 ms period)
/// - **Data Source**: Madgwick AHRS filter (IMU + magnetometer + barometer)
///
/// # Coordinate Frame
///
/// All orientation and position data uses the **NED (North-East-Down)** frame:
/// - Roll: Rotation about forward (X) axis
/// - Pitch: Rotation about right (Y) axis
/// - Yaw: Rotation about down (Z) axis
///
/// # Field Units and Ranges
///
/// | Field       | Unit         | Typical Range      | Notes                    |
/// |-------------|--------------|-------------------|--------------------------|
/// | `roll`      | degrees (°)  | -180.0 to +180.0  | Right wing down = +      |
/// | `pitch`     | degrees (°)  | -90.0 to +90.0    | Nose up = +              |
/// | `yaw`       | degrees (°)  | 0.0 to 360.0      | North = 0°, East = 90°   |
/// | `alt`       | meters (m)   | 0.0 to 5000.0     | AGL (Above Ground Level) |
/// | `speed`     | m/s          | 0.0 to 100.0      | Airspeed from pitot tube |
/// | `speed_z`   | m/s          | -50.0 to +50.0    | Vertical speed, down = + |
/// | `timestamp_us` | microseconds | 0 to u64::MAX  | System uptime            |
///
/// # Examples
///
/// ```no_run
/// use drone::types::Telemetry;
///
/// // Example telemetry during level flight
/// let telem = Telemetry {
///     roll: 0.0,           // Wings level
///     pitch: 5.0,          // Slight nose up
///     yaw: 90.0,           // Heading east
///     alt: 100.0,          // 100 meters AGL
///     speed: 25.0,         // 25 m/s airspeed (~90 km/h)
///     speed_z: -2.0,       // Climbing at 2 m/s
///     timestamp_us: 5_000_000, // 5 seconds after boot
/// };
/// ```
///
/// # Safety Notes
///
/// - **Stall Detection**: When `speed < 15.0` AND `pitch > 20.0`, the control
///   system detects a potential stall condition and increases throttle.
/// - **Altitude Reference**: `alt` is AGL (Above Ground Level), not MSL (Mean
///   Sea Level). Set ground reference during initialization.
///
/// # Related Types
///
/// - [`Command`]: Control outputs generated from this telemetry
/// - [`crate::sensor::fusion::SensorFusion`]: Produces telemetry data
/// - [`crate::control::pid::PidController`]: Consumes telemetry for control
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Telemetry {
    /// Roll angle in degrees (rotation about forward X-axis).
    ///
    /// - **Range**: -180.0° to +180.0°
    /// - **Convention**: Right wing down = positive, left wing down = negative
    /// - **Zero**: Wings level with horizon
    ///
    /// Derived from Madgwick AHRS quaternion using Euler angle conversion.
    pub roll: f32,

    /// Pitch angle in degrees (rotation about right Y-axis).
    ///
    /// - **Range**: -90.0° to +90.0°
    /// - **Convention**: Nose up = positive, nose down = negative
    /// - **Zero**: Fuselage level with horizon
    ///
    /// Critical for altitude hold and stall detection logic.
    pub pitch: f32,

    /// Yaw angle (heading) in degrees (rotation about down Z-axis).
    ///
    /// - **Range**: 0.0° to 360.0°
    /// - **Convention**: North = 0°, East = 90°, South = 180°, West = 270°
    /// - **Zero**: Magnetic north (from HMC5883L magnetometer)
    ///
    /// Requires magnetometer calibration for accurate heading reference.
    pub yaw: f32,

    /// Altitude in meters above ground level (AGL).
    ///
    /// - **Unit**: meters (m)
    /// - **Range**: 0.0 to ~5000.0 (practical ceiling)
    /// - **Source**: BMP180 barometer with ground pressure reference
    ///
    /// **Important**: This is AGL, not MSL. Ground reference pressure must be
    /// set during initialization or the first few seconds of flight.
    pub alt: f32,

    /// Horizontal airspeed in meters per second.
    ///
    /// - **Unit**: m/s
    /// - **Range**: 0.0 to ~100.0 (practical max)
    /// - **Source**: MS4525DO pitot tube (differential pressure sensor)
    ///
    /// Used for:
    /// - Speed hold PID loop
    /// - Stall detection (critical threshold: 15 m/s)
    /// - Dynamic pressure calculations
    pub speed: f32,

    /// Vertical speed (climb rate) in meters per second.
    ///
    /// - **Unit**: m/s
    /// - **Range**: -50.0 (descending) to +50.0 (climbing)
    /// - **Convention**: Positive = descending (NED down-positive), negative = climbing
    /// - **Source**: Time derivative of barometric altitude
    ///
    /// **Note**: Sign convention matches NED frame (down is positive), which is
    /// opposite of typical aviation convention. Negative values indicate climb.
    pub speed_z: f32,

    /// System timestamp in microseconds since boot.
    ///
    /// - **Unit**: microseconds (µs)
    /// - **Range**: 0 to u64::MAX (~584,000 years)
    /// - **Source**: Embassy async executor monotonic timer
    ///
    /// Used for:
    /// - Data freshness validation
    /// - Timeout detection (50 ms timeout triggers safe mode)
    /// - Performance profiling and loop timing analysis
    pub timestamp_us: u64,
}

/// Control commands for motors and flight control surfaces.
///
/// This structure contains the outputs from the PID control loops, intended
/// to drive the aircraft's actuators (EDF motors and servos). Values are
/// normalized or in degrees depending on the field.
///
/// # Update Rate
///
/// - **Control Task**: 100 Hz (10 ms period)
/// - **Outer Loop**: 50 Hz (altitude, speed)
/// - **Inner Loop**: 250 Hz (roll, pitch, yaw)
///
/// # Control Architecture
///
/// This follows a **dual-loop nested PID** design:
///
/// 1. **Outer Loop (50 Hz)**: Position/velocity control
///    - `alt` (pitch setpoint) ← Altitude PID
///    - `speed` (throttle) ← Speed PID
///
/// 2. **Inner Loop (250 Hz)**: Attitude control
///    - `roll` ← Roll PID (fast response, Kp=0.8)
///    - Pitch correction from outer `alt` setpoint
///    - `yaw` ← Yaw PID (Kp=0.5)
///
/// # Field Units and Ranges
///
/// | Field   | Unit/Type    | Range           | Actuator Target       |
/// |---------|--------------|-----------------|----------------------|
/// | `yaw`   | normalized   | -1.0 to +1.0    | Rudder servo         |
/// | `roll`  | normalized   | -1.0 to +1.0    | Aileron servos       |
/// | `alt`   | normalized   | -1.0 to +1.0    | Elevator servo       |
/// | `speed` | normalized   | 0.0 to 1.0      | EDF motor throttle   |
/// | `steer` | boolean      | true/false      | Undercarriage servo  |
///
/// # Examples
///
/// ```no_run
/// use drone::types::Command;
///
/// // Level flight at 50% throttle
/// let cmd = Command {
///     yaw: 0.0,      // No rudder input
///     roll: 0.0,     // Wings level
///     alt: 0.0,      // Neutral elevator
///     speed: 0.5,    // 50% throttle
///     steer: false,  // Steering closed (flight mode)
/// };
///
/// // Climbing right turn
/// let cmd_turn = Command {
///     yaw: 0.3,      // Rudder right
///     roll: 0.4,     // Bank right
///     alt: -0.2,     // Slight nose up
///     speed: 0.7,    // 70% throttle for climb
///     steer: false,
/// };
/// ```
///
/// # Actuator Mapping
///
/// ## Motors (DShot Protocol)
/// - **Field**: `speed`
/// - **Range**: 0.0 (idle) to 1.0 (full throttle)
/// - **Hardware**: 2× 70mm EDF motors (80A BLHeli ESCs)
/// - **Thrust**: 1.8 kg each @ full throttle (3.6 kg total)
/// - **Protocol**: DShot600 (0-2000 command range)
///
/// ## Servos (PWM @ 50 Hz)
/// - **Ailerons** (`roll`): ±90° servo travel, -1.0 = left down, +1.0 = right down
/// - **Elevator** (`alt`): ±30° travel limit, -1.0 = full down, +1.0 = full up
/// - **Rudder** (`yaw`): ±30° travel limit, -1.0 = full left, +1.0 = full right
/// - **Steering** (`steer`): true = open (ground mode), false = closed (flight mode)
///
/// # Safety Features
///
/// - **Servo Limits**: Hardware limits prevent damage from excessive angles
/// - **Throttle Clamping**: `speed` is clamped to 0.0-1.0 in DShot conversion
/// - **Stall Override**: When stall detected, `speed` boosted to 1.8× current value
/// - **Command Validation**: Control surface angles validated before PWM output
///
/// # Related Types
///
/// - [`Telemetry`]: Input to PID controllers that generate this command
/// - [`crate::control::pid::PidController`]: Generates command from telemetry
/// - [`crate::control::MotionController`]: Executes command on actuators
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Command {
    /// Yaw control output (rudder servo command).
    ///
    /// - **Range**: -1.0 (full left) to +1.0 (full right)
    /// - **Actuator**: Rudder servo (MG90S, ±30° travel)
    /// - **Source**: Yaw PID controller (Kp=0.5, Ki=0.02, Kd=0.1)
    ///
    /// Controls directional stability and coordinated turns. In coordinated
    /// flight, rudder compensates for adverse yaw during roll maneuvers.
    pub yaw: f32,

    /// Roll control output (aileron servo command).
    ///
    /// - **Range**: -1.0 (left down) to +1.0 (right down)
    /// - **Actuators**: Left and right aileron servos (MG90S, ±90° travel)
    /// - **Source**: Roll PID controller (Kp=0.8, Ki=0.05, Kd=0.15)
    ///
    /// Primary lateral control. Differential aileron deflection creates
    /// rolling moment about the aircraft's longitudinal axis.
    pub roll: f32,

    /// Altitude control output (elevator servo command / pitch setpoint).
    ///
    /// - **Range**: -1.0 (full down) to +1.0 (full up)
    /// - **Actuator**: Elevator servo (MG90S, ±30° travel)
    /// - **Source**: Outer loop altitude PID → pitch setpoint → inner pitch PID
    ///
    /// **Dual Purpose**:
    /// 1. Outer loop: Altitude error → pitch angle setpoint (±15° limit)
    /// 2. Inner loop: Pitch angle error → elevator deflection command
    ///
    /// Positive values pitch nose up, negative pitch nose down.
    pub alt: f32,

    /// Throttle control output (EDF motor command).
    ///
    /// - **Range**: 0.0 (idle) to 1.0 (full throttle)
    /// - **Actuators**: 2× 70mm EDF motors via DShot600 protocol
    /// - **Source**: Speed PID controller (Kp=0.1, Ki=0.02, Kd=0.05)
    ///
    /// Converted to DShot command range (0-2000) before transmission.
    /// **Stall override**: Automatically boosted to 1.8× when stall detected
    /// (speed < 15 m/s AND pitch > 20°).
    pub speed: f32,

    /// Undercarriage steering enable flag.
    ///
    /// - **Type**: Boolean
    /// - **Values**: `true` = steering open (ground ops), `false` = closed (flight)
    /// - **Actuators**: Main steering servo (MG996R) + nose wheel servo (MG955)
    ///
    /// Controls ground steering servos for taxi, takeoff, and landing operations.
    /// Automatically closed during flight to reduce drag and prevent servo flutter.
    pub steer: bool,
}
