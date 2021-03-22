# SLAM

### Description
SLAM(Simultaneous localization and mapping) from a motorcar implemented in python.

### Particle Filter SLAM
Simultaneous localization and mapping (SLAM) using odometry, 2-D LiDAR scans, and stereo camera measurements from an autonomous car. Odometry and LiDAR measurements are used to localize the robot and 2-D occupancy grid map of the environment is also build.

<img src="https://user-images.githubusercontent.com/15256774/111953680-4ce69800-8b2a-11eb-8da5-4f143843cd8f.gif" width="400" height="400"/><img src="https://user-images.githubusercontent.com/15256774/111953981-bff00e80-8b2a-11eb-8935-2e2b1d1d62e6.gif" width="400" height="400"/>
- \[left\] Real images over time
- \[right\] Estimated trajectory (green), particles (red), estimated empty space (white), and estimated objects (grey) over time by particle filter SLAM

- The vehicle is equipped with a variety of sensors. In this project, we will only use data from the front 2-D LiDAR scanner, fiber optic gyro (FOG), and encoders for localization and mapping as well as the stereo cameras for texture mapping. See Fig. 1 for an illustration.
  – The FOG provides relative rotational motion between two consecutive time stamps. The data can be used as ∆θ = τω, where ∆θ, ω, and τ are the yaw angle change, angular velocity, and time discretization, respectively.
  – The sensor data from the 2D LiDAR, encoders, and FOG are provided in .csv format. The first column in every file represents the timestamp of the observation. For the stereo camera images, each file is named based on the timestamp of the picture.
- The goal of the project is to use a particle filter with a differential-drive motion model and scan-grid correlation observation model for simultaneous localization and occupancy-grid mapping.

<img src="https://user-images.githubusercontent.com/15256774/111969836-000cbc80-8b3e-11eb-995d-fe5894239c1a.png" width="700"/>
Figure 1: Sensor layout on the autonomous vehicle. We will only use data from the wheel encoders, fiber optic gyro (FOG), front 2D LiDAR (Middle SICK), and the stereo cameras.
