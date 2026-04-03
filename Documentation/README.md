##  Documentation

Link to presentation **Airavat 1.0**:

<p align="center">
  <a href="https://pdpuacin-my.sharepoint.com/:p:/g/personal/23bme005_pdpu_ac_in/IQChsz3eY3VvR4gFlXudYxSFAX_xw7OShsRYDFWDMRPfu04?e=MHm1KI">
    <img src="assets/slide1.jpg" width="75%" alt="Airavat Presentation"/>
  </a>
</p>

<p align="center">
  <b>Click the image to view full presentation</b>
</p>

---

###  System Architecture

<p align="center">
  <img src="assets/slide5.jpg" width="75%" alt="System Architecture"/>
</p>

- 14.8V battery stepped down to 12V for power distribution  
- Raspberry Pi (onboard) handles data acquisition by interfacing with esp32 and sensors  
- Sensor data (LiDAR, IMU, odometry) is streamed to a ground station  
- Ground station performs SLAM, localization, and navigation (Nav2)  
- Computed velocity commands (`cmd_vel`) are sent back to the rover  
- ESP32 executes low-level motor control using encoder feedback  

---

###  SLAM (Mapping)

<p align="center">
  <img src="assets/slide8.jpg" width="75%" alt="SLAM Mapping"/>
</p>

---

###  Navigation Pipeline (ROS 2)

<p align="center">
  <img src="assets/slide9.jpg" width="75%" alt="Navigation Pipeline"/>
</p>

- Map used for localization and path planning  
- Global planner computes optimal path  
- Local planner handles real-time obstacle avoidance  
- Velocity commands sent to robot base  

---

### ⚙️ Control System

<p align="center">
  <img src="assets/slide10.jpg" width="75%" alt="Control System"/>
</p>

- Encoder-based closed-loop control  
- PID + feedforward for accurate motor actuation  
- Continuous feedback ensures stable motion  

---

