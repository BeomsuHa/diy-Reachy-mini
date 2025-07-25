# Reachy Mini Open Source Project

This repository contains the development progress for creating an open-source implementation of Reachy Mini, as the official version has not been fully released as open source yet (as of July 20, 2025).

<img src="images/reachy%20mini%20communication%20system.png" alt="reachy mini communication system" width="50%">

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

### Development Status
Tools/stewart_platform_simulator.py is the upper image
Script is built upon the [Stewart.js](https://github.com/rawify/Stewart.js)


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
