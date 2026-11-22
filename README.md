# Autonomous Driving Stack (ROS 2 & BeamNG.tech)

An end-to-end Advanced Driver Assistance Systems (ADAS) and autonomous driving software architecture developed in **C++**, **MATLAB**, and **ROS 2**. This project implements a complete perception, state estimation, and trajectory control pipeline designed for closed-loop testing in the **BeamNG.tech** simulation environment.

---

## Overview

The software stack processes raw stereo camera feeds and noisy vehicle telemetry from the simulator to reconstruct the road environment, detect obstacles, estimate the vehicle's true kinematic state, and compute optimal control commands in real time:

1. **Perception — Lane Extraction & Object Detection (OpenCV & RANSAC):**
   - Developed natively in **C++** using **OpenCV** (`StereoSGBM`), `message_filters`, and `cv_bridge`.
   - Synchronizes left and right stereo camera streams to compute dense disparity maps and reconstruct a 3D point cloud of the scene.
   - Applies **RANSAC (Random Sample Consensus)** to fit the dominant ground plane ($AX + BY + CZ + D = 0$) from 3D stereo points. Points deviating significantly above the estimated road plane are segmented as **3D objects/obstacles** (such as barriers, walls, and surrounding vehicles), while road-surface inliers are isolated for polynomial lane boundary fitting.
2. **State Estimation (Extended Kalman Filter):**
   - Prototyped in **MATLAB** and automatically translated into production **C++** code using the **MATLAB Coder** toolbox for seamless ROS 2 node integration.
   - Fuses multi-rate sensor data—including dual GNSS receivers, IMU, steering angle, and wheel speed sensors—under realistic simulated noise to continuously estimate vehicle pose, heading, and velocity.
3. **Trajectory Tracking (Model Predictive Control):**
   - Formulated as a **MATLAB** script and compiled into standalone **C++** code via **MATLAB Coder** to achieve deterministic, real-time execution within the ROS 2 control loop.
   - Uses the estimated vehicle state, obstacle constraints, and extracted reference trajectory to solve an optimization problem over a finite prediction horizon, outputting smooth steering and longitudinal control commands back to the vehicle.
4. **Configuration & Scenario Automation:**
   - All runtime parameters—including stereo camera intrinsics, sensor mounting extrinsics, RANSAC thresholds, EKF noise covariances, and MPC weight matrices (`Q`, `R`, `S`)—are decoupled from the C++ binaries and stored in structured **YAML** configuration files (`yaml-cpp`).
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
   ├── Stereo Cameras ──► [Vision Node (OpenCV & RANSAC)] ──► Lane Trajectory & 3D Objects
   └── GNSS / IMU / Wheel Encoders ──► [EKF Node (MATLAB Coder C++)] ──► Estimated Vehicle State
                                                                                │
   [Vehicle Actuators] ◄── Steering / Throttle ◄── [MPC Controller Node (MATLAB Coder C++)]
```

### Images from real-world scenarios and from simulator

Obstacle detection via machine vision
<img width="573" height="346" alt="image" src="https://github.com/user-attachments/assets/467ee964-af09-46ac-86f1-41fec98a7194" />

<img width="1048" height="590" alt="image" src="https://github.com/user-attachments/assets/6b968bea-ffee-4d5a-bc38-8917e1de62fb" />

<img width="998" height="589" alt="image" src="https://github.com/user-attachments/assets/8b8034f8-ad2d-47b7-84f0-5614583c0f35" />

### Image of Software architecture design

<img width="793" height="593" alt="image" src="https://github.com/user-attachments/assets/8616275a-380e-44ed-a303-ff67bb348f5f" />
