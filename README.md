# AV with Localization and Navigation

This project implements an **autonomous vehicle using ROS2 Humble**.  
The repository contains two main packages: `camera_viewer` and `localization`.  
The robot performs **perception, localization, path planning, and obstacle avoidance** to autonomously navigate and deliver objects.

---

## 📌 Features

- **Camera streaming** on Raspberry Pi + Jetson Nano via ROS2.
- **Localization** with LiDAR + ArUco markers + Kalman filter correction.
- **Navigation**:
  - Finds a cube.
  - Approaches and picks it up.
  - Delivers the cube to point A, B, or C (based on ArUco marker).
  - Calculates routes and avoids obstacles.
- **ROS2 Humble integration** with differential drive robot.
