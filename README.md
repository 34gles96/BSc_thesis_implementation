# Autonomous Driving Stack (ROS 2 & BeamNG.tech)

An end-to-end Advanced Driver Assistance Systems (ADAS) and autonomous driving software architecture developed in **C++**, **MATLAB**, and **ROS 2**. This project implements a complete perception, state estimation, and trajectory control pipeline designed for closed-loop testing in the **BeamNG.tech** simulation environment.

---

## Overview

The software stack processes raw stereo camera feeds and noisy vehicle telemetry from the simulator to reconstruct the road environment, estimate the vehicle's true kinematic state, and compute optimal control commands in real time:

1. **Perception (Stereo Machine Vision):**
   - Developed natively in C++ using OpenCV (`StereoSGBM`), `message_filters`, and `cv_bridge`.
   - Synchronizes left and right stereo camera streams, computes dense disparity maps, projects detected lane markings into 3D space, filters out elevated obstacles (such as walls and barriers) via road-plane estimation ($AX + BY + CZ + D = 0$), and fits second-order polynomials to extract the drivable lane trajectory.
2. **State Estimation (Extended Kalman Filter):**
   - Prototyped in **MATLAB** and automatically translated into production **C++** code using the **MATLAB Coder** toolbox for seamless ROS 2 node integration.
   - Fuses multi-rate sensor data—including dual GNSS receivers, IMU, steering angle, and wheel speed sensors—under realistic simulated noise to continuously estimate vehicle pose, heading, and velocity.
3. **Trajectory Tracking (Model Predictive Control):**
   - Formulated as a **MATLAB** script and compiled into standalone **C++** code via **MATLAB Coder** to achieve deterministic, real-time execution within the ROS 2 control loop.
   - Uses the estimated vehicle state and extracted reference trajectory to solve an optimization problem over a finite prediction horizon, outputting smooth steering and longitudinal control commands back to the vehicle.
4. **Configuration & Scenario Automation:**
   - All runtime parameters—including stereo camera intrinsics, sensor mounting extrinsics, EKF noise covariances, and MPC weight matrices (`Q`, `R`, `S`)—are decoupled from the C++ binaries and stored in structured **YAML** configuration files (`yaml-cpp`).
   - Includes **helper scripts** that parse **BeamNG.tech** scenario JSON files to automatically generate matching YAML configuration files, ensuring sensor setups and simulation parameters stay synchronized across different test scenarios without manual edits.

---

## Prerequisites & Simulation Requirements

> **Important:** **BeamNG.tech** is required to run and test this project in a closed-loop simulation. Without an active BeamNG.tech instance and its ROS 2 bridge, the perception, estimation, and control nodes will not receive sensor streams or vehicle feedback.

* **Simulator:** [BeamNG.tech](https://beamng.tech/) (academic or commercial license required) with the BeamNG-ROS2 bridge configured.
* **Middleware:** ROS 2 (Humble or Jazzy)
* **Development & Code Generation Tools:**
  * **MATLAB & MATLAB Coder** (used to generate the C++ EKF and MPC libraries from the source MATLAB scripts; the generated C++ files compile directly via `colcon` without requiring a live MATLAB runtime during simulation)
  * C++17 compiler & `colcon` build system
  * OpenCV (`cv_bridge`, `image_transport`, `message_filters`)
  * `yaml-cpp` (for parsing runtime configuration files)

---

## Architecture Pipeline

```text
[BeamNG.tech Scenario JSON] ──► [Helper Scripts] ──► [YAML Config Files]
                                                            │ (parameters)
[BeamNG.tech Simulator]                                     ▼
   ├── Stereo Cameras ──► [Stereo Vision Node (C++ / OpenCV)] ──► 3D Road & Lane Trajectory
   └── GNSS / IMU / Wheel Encoders ──► [EKF Node (MATLAB Coder C++)] ──► Estimated Vehicle State
                                                                                │
   [Vehicle Actuators] ◄── Steering / Throttle ◄── [MPC Controller Node (MATLAB Coder C++)]