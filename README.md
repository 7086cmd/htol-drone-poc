# htol-drone-poc

A Horizontal Take-off and Landing Drone, Proof of Concept.

## Overview

This project serves as a proof of concept for a horizontal take-off and landing (HTOL) drone. The goal is to design and build a drone that can take off and land horizontally, similar to traditional aircraft. This approach aims to combine the advantages of both fixed-wing and rotary-wing drones, providing improved efficiency and versatility.

Drones nowadays are mostly VTOL (Vertical Take-off and Landing) types, which have limitations in terms of speed, range, and energy efficiency. By exploring HTOL designs, we hope to overcome these limitations and open up new possibilities for drone applications. Furthermore, isn't it exciting to see a drone take off and land like a plane?

## Designs

- Plane Body: The drone can be 3D printed using LW-PLA material in about 20% to 30% infill to ensure durability while keeping the weight low.
- Wing Configuration: The wings are designed to provide optimal lift and stability during flight. They can be printed separately and attached to the main body using screws or adhesive.
- Power system:
   - Motors: The drone is powered by two brushless motors, that balancedly placed on each wing, providing sufficient thrust for take-off, flight, and landing. Currently I prefer the A2212 motors in 1000 KV variant, with 1060 propellers.
   - Servos: For control surfaces, standard servos are used to manipulate the ailerons, elevator, and rudder.
      - Ailerons and Elevator: MG90S servos are used for controlling the ailerons and elevator, providing precise control over roll and pitch movements.
      - Rudder: SG5010 servos are used for controlling the rudder, allowing for effective yaw control.
      - Nose Wheel Steering: A MG955 servo is used for steering the nose wheel during taxiing, take-off, and landing. This ensures durable and responsive control of the nose wheel in preventing it from crashing.
   - ESCs: Electronic Speed Controllers (ESCs) are used to regulate the speed of the brushless motors. We recommend 30A ESCs to ensure smooth and responsive control.
   - Power Supply: ACE RC 3S 5300 mAh LiPo batteries are used to provide sufficient power for the drone's flight duration and performance. Further more, two 18650 Li-ion batteries are used to power the onboard electronics. Concerning its weight, a BEC (Battery Eliminator Circuit) can be used to directly step down the voltage from the main battery to power the electronics, instead of using separate batteries.
- Flight Controller:
   - Lower Layer: The drone is equipped with a STM32H743VGT6 microcontroller, which serves as the flight controller. It is responsible for stabilizing the drone, managing flight modes, and processing sensor data. (*That's because only STM32H743VGT6 in my stock.*)
   - Upper Layer: We leverage Orange Pi Zero 3 (with 4GB RAM) as the companion computer to handle high-level tasks such as mission planning, computer vision, and communication with ground control stations. (*That's because only Orange Pi Zero 3 in my stock.*)
   - Optional Phone Integration: Using the Global Mobile Service (GMS) capabilities on phones, we can put smartphones into the drone for GMS-based navigation and control. This allows for additional functionalities such as real-time video streaming, telemetry data display, and remote control via mobile apps.
   - NVIDIA Jetson Orin Nano 4GB could also be integrated for further AI-based functionalities, such as object detection and autonomous navigation. But it's not in the first version due to budget constraints and weight consider it in future iterations.
- Sensors: Various sensors are integrated into the drone to enhance its flight capabilities. These include an IMU (Inertial Measurement Unit) for attitude estimation, a barometer for altitude measurement, and a GPS module for navigation.
   - IMU: MPU6050 (with BNO055 for data fusion if needed)
   - Barometer: BMP280
   - GPS: NEO-6M
- Communication: The drone utilizes a 2.4GHz radio frequency (RF) communication system for remote control and telemetry data transmission. This allows for real-time monitoring and control of the drone during flight.
- Software: The flight control software is developed using Rust programming language, leveraging its safety and performance features. The software implements various flight control algorithms, sensor fusion techniques, and communication protocols to ensure stable and reliable flight performance.

## Technological Details

The project utilizes a combination of hardware and software technologies to achieve the desired HTOL capabilities. The hardware components include brushless motors, servos, ESCs, batteries, flight controllers, sensors, and communication modules. The software stack is built using Rust programming language, which provides safety and performance benefits. The flight control algorithms are implemented using a combination of PID control, sensor fusion techniques, and state estimation methods.

1. Brushless Motors and ESCs: The drone is powered by brushless motors, which provide high efficiency and power-to-weight ratio. The ESCs regulate the speed of the motors based on control signals from the flight controller. With the help of PWM (Pulse Width Modulation) signals, the ESCs can precisely control the motor speed, allowing for smooth acceleration and deceleration during flight. In a 50 Hz period, 1000 microseconds pulse width represents zero throttle, while 2000 microseconds represents full throttle. DShot and OneShot protocols could also be implemented for better performance. It seems that the crate `dshot-frame` could be useful for DShot implementation.
2. Servos: The servos are more straightforward to control using PWM signals. Using the `SimplePwm` or `ComplementaryPwm` from the `embassy-stm32` crate, we can generate the required PWM signals to control the servo positions. For example, a pulse width of 1000 microseconds could represent the minimum position, while 2000 microseconds represents the maximum position. The servos respond to these signals by adjusting their angles accordingly.
3. Sensor Integration: The drone integrates various sensors such as IMU, barometer, and GPS to provide accurate flight data. The IMU provides information about the drone's orientation and angular velocity, while the barometer measures altitude. The GPS module provides position data for navigation. Sensor fusion techniques, such as a complementary filter or Kalman filter, are used to combine data from multiple sensors to improve accuracy and reliability. The IMU and barometer uses I2C protocol for communication, while the GPS module uses UART protocol.
4. Flight Control Algorithms: The flight control algorithms can be initially implemented using PID (Proportional-Integral-Derivative) control to stabilize the drone during flight. The PID controller takes the desired setpoints (e.g., roll, pitch, yaw angles) and compares them with the actual measurements from the sensors. Based on the error between the setpoints and measurements, the PID controller calculates control outputs to adjust motor speeds and servo positions accordingly. More advanced control algorithms, such as model predictive control or adaptive control, could be explored in future iterations.

## Subsystems Design

The HTOL drone is composed of several subsystems that work together to achieve horizontal take-off and landing capabilities. These subsystems include:

- Propulsion System: The propulsion system consists of brushless motors, ESCs, and propellers. The motors provide the necessary thrust for take-off, flight, and landing. The ESCs regulate the speed of the motors based on control signals from the flight controller. The propellers are designed to optimize thrust and efficiency during horizontal flight.
- Flight Control System: The flight control system is responsible for stabilizing the drone, managing flight modes, and data fusion from various sensors. The STM32H743VGT6 microcontroller serves as the primary flight controller, while the Orange Pi Zero 3 acts as a companion computer for high-level tasks, as well as LiDAR processing if needed. It analyzes sensor data, executes flight control algorithms, and sends control signals to the propulsion system and servos. It controls the speed, roll, yaw, and altitude of the drone during flight.
   For example, to ascend the drone, the flight controller increases the throttle of the motors, generating more lift. To change direction, it adjusts the ailerons, elevator, and rudder using the servos.
- Sensor System: The sensor system includes an IMU, barometer, and GPS module. These sensors provide essential data for flight control and navigation. The IMU measures the drone's orientation and angular velocity, the barometer measures altitude, and the GPS module provides position data. The sensor data is processed and fused to improve accuracy and reliability.
- Communication System: The communication system enables remote control and telemetry data transmission. By using GMS capabilities on smartphones, we can let the drone stay connected in anywhere with cellular coverage, advanced by the low-lantency 5G networks. The 2.4GHz RF communication system allows for real-time monitoring and control of the drone during flight. We can even consider using Ethernet for high-speed data transfer between the Orange Pi Zero 3 and the STM32 microcontroller.
   A communication protocol should be implemented among smartphones and the controller, which actually resembles pilots and the air traffic control tower in real life. This protocol should cover commands for take-off, landing, navigation, and emergency procedures.
   We have an abstraction on it. We classify the communication into three types:
   1. Request/response. The pilot requests to ascent, for example, to the ATC, and the ATC responds with an acknowledgment or denial (sometimes with a negotiated altitude instead).
   2. Information broadcast. The ATC broadcasts weather updates, no-fly zones, and other relevant information to all pilots in the area.
   3. Emergency alerts. In case of emergencies, such as loss of communication or system failures, the ATC can send emergency alerts to all pilots, instructing them on the necessary actions to take.
- Power System: The power system consists of LiPo batteries that provide the necessary energy for the drone's operation. The batteries are selected to ensure sufficient flight duration and performance. The power system also includes voltage regulators and power distribution components to ensure stable and reliable power delivery to all subsystems.

## Aerodynamics

We list components and its weights and sizes as well as the estimated weight and size of the entire drone.

We design the wingspan to be around 1.2 meters, providing sufficient lift for horizontal take-off and landing. The wing area is optimized to balance lift and drag, ensuring efficient flight performance. The airfoil shape is selected to enhance lift generation while minimizing drag.

| Component          | Qty | Weight (grams) | Size (mm)               | Model                   |
|--------------------|-----|----------------|-------------------------|-------------------------|
| Plane Body         | 1   | TBA            | TBA                     | TBA                     |
| Wings              | 1   | TBA            | TBA                     | TBA                     |
| Motors             | 2   | 51.5 g         | 27.7 * 27               | A2212 1000KV            |
| Propellers         | 2   | 1.29 g         | 254 (l) / 124 (h) φ6    | APC 10x6E               |
| ESCs               | 2   | 30 g           | 45 * 24 * 9             | 30A ESC                 |
| Servos (MG995)     | 1   | 55 g           | 54 * 45 * 20            | MG995                   |
| Servos (SG5010)    | 2   | 39 g           | 44 * 40 * 20            | SG5010                  |
| Servos (MG90S)     | 3   | 14 g           | 32 * 23 * 12            | MG90S                   |
| IMU (MPU6050...).  | 1   | 5 g            | 22 * 17 * 4             | MPU6050 and more        |
| GPS (NEO-6M)       | 1   | 12 g           | 25 * 25 * 7             | NEO-6M                  |
| STM32H743VGT6      | 1   | TBA            | 63 * 36                 | STM32H743VGT6 Dev Board |
| OrangePi Zero 3    | 1   | 24 g           | 55 * 50                 | Orange Pi Zero 3        |
| ACE RC 3S 5300mAh  | 1   | 340 g          | 29 * 43 * 136           | ACE RC LiPo Battery     |
| Carbon Fiber Rods  | TBA | TBA            | TBA                     | TBA                     |

Note: The IMU module includes MPU6050, BNO055, and BMP280 sensors.

## Extensions and Future Work

### Autonomous Flight

Given my prior experience in latent reasoning on large language models, it's intriguing to consider the potential for autonomous flight capabilities in this HTOL drone. By integrating advanced AI algorithms and leveraging the computational power of the onboard companion computer (Orange Pi Zero 3), we can explore the possibility of enabling the drone to perform autonomous take-offs, landings, and navigation.

This would involve implementing computer vision techniques for obstacle detection and avoidance, as well as path planning algorithms to navigate through complex environments. The integration of AI models could also facilitate real-time decision-making, allowing the drone to adapt to changing conditions during flight.

### Electromagnetic Catapult Launch System

To further enhance the take-off capabilities of the HTOL drone, we can explore the development of an electromagnetic catapult launch system. This system would utilize electromagnetic forces to propel the drone into the air, providing a more efficient and controlled take-off mechanism. The catapult system could be designed to work in conjunction with the drone's existing propulsion system, allowing for a smooth transition from launch to powered flight.

For the landing phase, we can consider implementing a retractable landing gear system that can be deployed just before touchdown. This would help to absorb the impact of landing and protect the drone's structure. Additionally, we could explore the use of parachutes or air brakes to further slow down the drone during descent, ensuring a safe and controlled landing.

## Disclaimer

**This is a personal learning project and experimental proof-of-concept, not a production-ready design or a complete engineering specification.**

### What This Project Is

- **An exploration**: I'm investigating HTOL (Horizontal Take-Off and Landing) drone concepts because the mechanics are interesting and different from typical VTOL drones
- **A learning platform**: This serves as my testbed for embedded Rust development, flight control algorithms, sensor fusion, and hardware integration
- **Iterative development**: The design will evolve through testing and experimentation. Many components are marked "TBA" because they'll be determined through actual prototyping
- **For fun**: The primary goal is to learn and build something technically challenging, not to create a commercial product

### What This Document Is Not

- **Not a complete engineering specification**: Many details (weight estimates, aerodynamic calculations, structural analysis) are intentionally left for the implementation phase
- **Not a guaranteed-to-fly design**: This is a starting point. Real-world testing will inevitably reveal issues requiring redesign
- **Not a beginner's guide**: This assumes familiarity with embedded systems, electronics, and basic aerodynamics
- **Not claiming novelty**: HTOL drones exist; this is my implementation using technologies I want to learn (Rust, specific hardware platforms, etc.)

### Development Approach

This project follows an incremental development methodology:

1. **Phase 1**: Airframe design and basic electronics integration
2. **Phase 2**: Flight controller firmware (Rust-based, using embassy-stm32)
3. **Phase 3**: Initial flight testing and tuning
4. **Phase 4**: Higher-level autonomy features (Orange Pi integration)
5. **Future extensions**: Advanced features listed in the "Extensions" section are aspirational and may never be implemented

Components marked "TBA" are placeholders to be determined through testing, not oversights in planning.

### Technical Challenges Acknowledged

I'm aware that:
- Building a flight controller from scratch is complex and time-consuming
- Weight and power budgets will need validation through actual hardware
- Aerodynamic performance will need empirical testing and iteration  
- First flight attempts may not succeed; this is expected and part of the learning process
- Some design decisions may prove suboptimal and require revision

### Safety Notice

**This project involves LiPo batteries, spinning propellers, and flying objects.** If you attempt to build something similar:
- Follow proper LiPo battery safety procedures
- Test in open areas away from people and property
- Comply with local UAV/drone regulations
- Ensure adequate liability insurance where required
- Start with ground testing before any flight attempts

### Contributions and Feedback

Technical feedback, suggestions, and corrections are welcome. However, please understand:
- This is a personal project progressing at my own pace
- Design choices reflect my learning goals, not necessarily optimal engineering
- I may choose approaches that are "interesting" over "established best practice" intentionally

**In short**: This is a toy project for learning. It's technically ambitious, may not work as initially designed, and that's perfectly fine. If you're looking for a proven, ready-to-build drone design, this isn't it. If you're interested in the journey of building complex embedded systems from scratch, follow along!
