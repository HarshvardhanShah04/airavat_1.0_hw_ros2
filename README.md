# 🚀 Airavat 1.0 — Autonomous Rover (ROS 2)

Airavat 1.0 is a ROS 2-based autonomous ground rover that demonstrates a complete pipeline from hardware-level sensing (Raspberry Pi) to autonomous navigation (SLAM + Nav2) on a ground station.  
Airavat was started as to validated our learnings and the software stack. I preferred a sim to real pipeline and this is the final hardware implementation of the airavat_1.0_sim_ros2.

---

## What This Project Covers

* Sensor data acquisition (IMU + wheel encoders)
* Odometry estimation
* Sensor fusion using EKF
* SLAM (mapping)
* Localization (AMCL)
* Autonomous navigation (Nav2)

---

##  System Architecture

### Rover (Raspberry Pi)

Responsible for hardware interfacing:

* Publishes IMU data (BNO055)
* Computes wheel odometry
* Streams sensor data over ROS 2

### Ground Station (Laptop)

Handles autonomy and decision-making:

* Sensor fusion (EKF)
* SLAM for map generation
* Localization using AMCL
* Path planning & execution (Nav2)
* Visualization in RViz

---

## 📂 Repository Structure

```
src/
├── GroundStation-Laptop/
│   ├── airavat_bringup/        # Launch files (SLAM, Nav2)
│   ├── airavat_description/    # URDF + RViz
│   ├── airavat_nav2/           # Navigation configs
│   ├── airavat_slam/           # SLAM toolbox
│   └── ekf_node/               # Sensor fusion

├── Rover-RPI/
│   ├── imu_publisher/          # IMU node
│   └── wheel_interface/        # Odometry node
```

---

## 🎥 Demo & Documentation

Additional resources are available in:

```
Documentation/
```

Includes:

* System presentation
* Motor tuning data

---

## Hardware

* Raspberry Pi (Ubuntu 22.04 + ROS 2)
* Laptop as groundstation (Ubuntu 22.04 + ROS 2)
* BNO055 IMU
* Differential drive rover with encoders

---

## Repository Notes

This repository focuses on core robotics functionality.
Non-essential files such as logs, cache files, and backups have been excluded for clarity.

---

##  Author

Harsh
Robotics | ROS 2 | Autonomous Systems

