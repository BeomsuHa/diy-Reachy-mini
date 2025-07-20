# Reachy Mini Open Source Project

This repository contains the development progress for creating an open-source implementation of Reachy Mini, as the official version has not been fully released as open source yet (as of July 20, 2025).

## Project Overview

This project aims to build a comprehensive control system for Reachy Mini through modular development, focusing on four main components:

1. **Head Control via Stewart Platform**
2. **Antenna Control**
3. **Full Body Rotation Control**
4. **Camera Module Control**

---

## 1. Head Control via Stewart Platform

![Stewart Platform Simulator](images/Stewart%20Platform%20SImulatior%20image.png)

### Approach
The head movement control is implemented using a pre-calculated pose-based motor rotation method. This approach utilizes inverse kinematics calculations to determine the precise motor positions required for desired head orientations.

### Implementation
The system is built upon the [Stewart.js library](https://github.com/rawify/Stewart.js) (RAW inverse kinematics library for Stewart Platforms written in JavaScript), which has been adapted and ported to Python to create a comprehensive simulator.

#### Key Features:
- **Inverse Kinematics Engine**: Converts desired head pose (position and orientation) to individual actuator lengths
- **Real-time Simulation**: Python-based simulator for testing and validation
- **Motor Control Interface**: Direct communication with servo motors based on calculated positions
- **Pose Validation**: Ensures requested poses are within the platform's mechanical limits

#### Technical Stack:
- **Language**: Python
- **Architecture**: Class-based modular structure
- **Dependencies**: NumPy for mathematical calculations, custom inverse kinematics solver
- **Hardware Interface**: Servo motor control via PWM signals

### Development Status
🚧 **In Progress** - Core inverse kinematics implementation completed, motor integration in development

---

## 2. Antenna Control
🔄 **Planned** - Development scheduled after head control completion

---

## 3. Full Body Rotation Control
🔄 **Planned** - Development scheduled after antenna control completion

---

## 4. Camera Module Control
🔄 **Planned** - Final component to be implemented

## License

MIT License

## Acknowledgments

- **Stewart.js**: Original JavaScript implementation of Stewart Platform inverse kinematics
- **Pollen Robotics**: Original creators of Reachy Mini

---

**Note**: This is an independent open-source implementation and is not officially affiliated with Pollen Robotics.