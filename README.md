# Wheel Loader Robot Control System

**An autonomous wheel loader robot system built on PX4 autopilot framework**

This repository contains a specialized control system for autonomous wheel loader robots, based on the PX4 autopilot platform. The system provides comprehensive control capabilities for heavy-duty wheel loader operations including:

- **Boom and Bucket Control**: Precise hydraulic actuator control for boom kinematics and bucket operations
- **Autonomous Navigation**: Advanced path planning and obstacle avoidance for construction environments  
- **Load Management**: Intelligent load detection, pickup, and placement algorithms
- **Safety Systems**: Multi-layered safety monitoring with emergency stop capabilities
- **Dual-Mode Operation**: Support for both manual operator control and fully autonomous operation
- **Real-time Monitoring**: Telemetry and status reporting for remote monitoring and fleet management

The system is designed for industrial applications in construction, mining, and material handling environments, providing reliable autonomous operation while maintaining the safety standards required for heavy equipment.

---

## Original PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Build Targets](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml/badge.svg?branch=main)](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High Altitude Balloons, Spacecraft, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)

## Releases

Release notes and supporting information for PX4 releases can be found on the [Developer Guide](https://docs.px4.io/main/en/releases/).

## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4. See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing Code and Contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

For the latest stats on contributors please see the latest stats for the Dronecode ecosystem in our project dashboard under [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode). For information on how to update your profile and affiliations please see the following support link on how to [Complete Your LFX Profile](https://docs.linuxfoundation.org/lfx/my-profile/complete-your-lfx-profile). Dronecode publishes a yearly snapshot of contributions and achievements on its [website under the Reports section](https://dronecode.org).

## Supported Hardware

For the most up to date information, please visit [PX4 User Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>

---

## Wheel Loader Robot Specific Features

### Supported Hardware Configurations

This wheel loader control system supports the following hardware configurations:

- **CUAV X7Plus-WL**: Wheel loader variant with enhanced I/O for hydraulic control
- **HKUST NXT-Dual-WL-Front**: Front-mounted control unit for dual-controller setups  
- **HKUST NXT-Dual-WL-Rear**: Rear-mounted control unit for comprehensive vehicle control

### Building for Wheel Loader Targets

```bash
# Build for CUAV X7Plus wheel loader variant
make cuav_x7plus-wl_default

# Build for HKUST NXT dual wheel loader configurations
make hkust_nxt-dual-wl-front_default
make hkust_nxt-dual-wl-rear_default
```

### Key Wheel Loader Modules

- **Boom Control Module**: Advanced kinematic control for boom positioning
- **Bucket Control Module**: Hydraulic control for bucket tilt and curl operations
- **Load Detection**: Weight and load distribution sensing
- **Safety Monitor**: Continuous monitoring of hydraulic pressure, temperature, and system status
- **Autonomous Navigation**: GPS-based waypoint navigation with obstacle avoidance

### Documentation

Additional wheel loader specific documentation can be found in:
- [Boom Kinematics Design](docs/BOOM_KINEMATICS_DESIGN.md)
- [Bucket Control Design](docs/BUCKET_KINEMATICS_DESIGN.md) 
- [VLA Dual Mode Robot](docs/VLA_Dual_Mode_Robot.md)
- [Wheel Loader Robot Design](design/wheel_loader_robot_design.md)

### Version Information

Current wheel loader robot system version: **v0.1.0**
Based on PX4 Autopilot framework with specialized wheel loader extensions.
