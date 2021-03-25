# SLAM

## Description
[SLAM(Simultaneous localization and mapping)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) implemented in python.

## Particle Filter SLAM
### Overview
Simultaneous localization and mapping (SLAM) based on [particle filter](https://en.wikipedia.org/wiki/Particle_filter#:~:text=Particle%20filtering%20uses%20a%20set,can%20take%20any%20form%20required.) using [odometry](https://en.wikipedia.org/wiki/Odometry), 2-D [LiDAR](https://en.wikipedia.org/wiki/Lidar) scans, and [stereo camera](https://en.wikipedia.org/wiki/Stereo_camera) measurements from an autonomous car. Odometry and LiDAR measurements are used to localize the robot and 2-D [occupancy grid map](https://en.wikipedia.org/wiki/Occupancy_grid_mapping) of the environment is also build.

<img src="https://user-images.githubusercontent.com/15256774/111973305-b32ae500-8b41-11eb-88eb-17f68ab933e9.gif" width="400" height="400"/><img src="https://user-images.githubusercontent.com/15256774/111973334-baea8980-8b41-11eb-9c7b-ba07595593a4.gif" width="400" height="400"/>
- \[left\] Real images over time
- \[right\] Estimated trajectory (green), particles (red), estimated empty space (white), and estimated objects (grey) over time by particle filter SLAM

- The vehicle is equipped with a variety of sensors. In this project, we will only use data from the front 2-D LiDAR scanner, [fiber optic gyro (FOG)](https://en.wikipedia.org/wiki/Fibre-optic_gyroscope), and encoders for localization and mapping as well as the stereo cameras for texture mapping. See Fig. 1 for an illustration.
  - The FOG provides relative rotational motion between two consecutive time stamps. The data can be used as ∆θ = τω, where ∆θ, ω, and τ are the yaw angle change, angular velocity, and time discretization, respectively.
  - The sensor data from the 2D LiDAR, encoders, and FOG are provided in .csv format. The first column in every file represents the timestamp of the observation. For the stereo camera images, each file is named based on the timestamp of the picture.
  - The goal of the project is to use a particle filter with a differential-drive motion model and scan-grid correlation observation model for simultaneous localization and occupancy-grid mapping.

<img src="https://user-images.githubusercontent.com/15256774/111969836-000cbc80-8b3e-11eb-995d-fe5894239c1a.png" width="700"/>
Figure 1: Sensor layout on the autonomous vehicle. We will only use data from the wheel encoders, fiber optic gyro (FOG), front 2D LiDAR (Middle SICK), and the stereo cameras.

### Technical Details
#### Lidar data
The Lidar data is collected in 190 degree (from -5 to $185$ degree) with resolution 0.666. We transformed the coordinates of objects in Lidar frame to the body frame using rotation matrix R and translation vector T,

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0Az_%7Bbody%7D+%3D+R%28z_%7BLidar%7D+%2B+T%29%0A%5Cend%7Balign%2A%7D%0A" 
alt="\begin{align*}
z_{body} = R(z_{Lidar} + T)
\end{align*}
">

#### Particle filter
Particle filter consist of three steps: prediction, update and resampling.  
  
(i) Prediction  
In prediction, each particle predicts the location, velocity, angular velocity at time t+1 based on the estimated location and distributions of linear velocity and angular velocity at time t denoted as
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0Ap%28v_%7Bt%2B1%7D%2C%5Comega_%7Bt%2B1%7D+%7C+v_%7Bt%7D%2C%5Comega_%7Bt%7D%29%0A%5Cend%7Balign%2A%7D%0A" 
alt="\begin{align*}
p(v_{t+1},\omega_{t+1} | v_{t},\omega_{t})
\end{align*}
">
where
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%28v_%7Bt%2B1%7D%2C+%5Comega_%7Bt%2B1%7D%29%5ET+%5Csim+%5Cmathcal%7BN%7D%28%5Cmu_%7Bt%7D%2C+%5CSigma_%7Bt%7D%29%0A%5Cend%7Balign%2A%7D%0A" 
alt="\begin{align*}
(v_{t+1}, \omega_{t+1})^T \sim \mathcal{N}(\mu_{t}, \Sigma_{t})
\end{align*}
">
where $\mu_t$ is expected value of $v$ and $\omega$ at time $t$ and $\Sigma_{t}$ is covariance matrix of these two. These are equal to sample mean and sample covariance matrix in the previous step $t$. We assume the machine moves in constant velocity during a sufficiently small period $[t-1, t)$ and $v_t$ and $\omega_t$ is normally distributed in such small period. Based on these assumption, we sampled $v_{t+1}$ and $\omega_{t+1}$ according to $p(v_{t+1},\omega_{t+1} | v_{t},\omega_{t})$, or pdf of two dimensional normal distribution more precisely, for each particle.

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%5Cleft%28%5Cbegin%7Barray%7D%7Bc%7D%0Ax_%7Bi%2C+t%2B1%7D%5C%5C%0Ay_%7Bi%2C+t%2B1%7D%5C%5C%0A%5Ctheta_%7Bi%2C+t%2B1%7D%0A%5Cend%7Barray%7D%5Cright%29%0A%5Cleftarrow%0A%5Cleft%28%5Cbegin%7Barray%7D%7Bc%7D%0Ax_%7Bi%2C+t%7D%5C%5C%0Ay_%7Bi%2C+t%7D%5C%5C%0A%5Ctheta_%7Bi%2Ct%7D%0A%5Cend%7Barray%7D%5Cright%29%0A%2B%0A%5Cleft%28%5Cbegin%7Barray%7D%7Bc%7D%0Av+%5Ccos%7B%5Ctheta_%7Bi%2Ct%7D%7D%5C%5C%0Av+%5Csin%7B%5Ctheta_%7Bi%2Ct%7D%7D%5C%5C%0A%5Comega_%7Bi%2Ct%7D%0A%5Cend%7Barray%7D%5Cright%29%0A%5Cend%7Balign%2A%7D%0A" 
alt="\begin{align*}
\left(\begin{array}{c}
x_{i, t+1}\\
y_{i, t+1}\\
\theta_{i, t+1}
\end{array}\right)
\leftarrow
\left(\begin{array}{c}
x_{i, t}\\
y_{i, t}\\
\theta_{i,t}
\end{array}\right)
+
\left(\begin{array}{c}
v \cos{\theta_{i,t}}\\
v \sin{\theta_{i,t}}\\
\omega_{i,t}
\end{array}\right)
\end{align*}
">

where $i = 1,2,...n$ denotes each particle. Then, we estimate the location of the machine by computing weighted sum of estimate of particles,

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+z_%7Bt%2B1%7D+%3D+%5Csum%5En_%7Bi%3D1%7Dw_%7Bi%2Ct%2B1%7Dz_%7Bi%2Ct%2B1%7D%2C%5C+%5C+%5C+z_t+%3D+%0A%5Cleft%28%5Cbegin%7Barray%7D%7Bc%7D%0Ax_%7Bi%2C+t%7D%5C%5C%0Ay_%7Bi%2C+t%7D%5C%5C%0A%5Ctheta_%7Bi%2Ct%7D%0A%5Cend%7Barray%7D%5Cright%29" 
alt="z_{t+1} = \sum^n_{i=1}w_{i,t+1}z_{i,t+1},\ \ \ z_t = 
\left(\begin{array}{c}
x_{i, t}\\
y_{i, t}\\
\theta_{i,t}
\end{array}\right)">

(ii) Update  
In update step, weight of each particle is updated based on value of map correlation between a localized created by the particle and a global map as follows. Details of map correlation will be discussed later.

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+w_%7Bi%2Ct%2B1%7D+%3D+%5Cfrac%7Bcorr_%7Bi%2Ct%7D%7D%7B%5Csum%5En_%7Bj%3D1%7Dcorr_%7Bj%2Ct%7D%7D" 
alt="w_{i,t+1} = \frac{corr_{i,t}}{\sum^n_{j=1}corr_{j,t}}">

(iii) Resampling  
If the number of particle whose weight is smaller than some pre-set threshold, $w_{threshold}$ becomes some pre-set number, $n_{small\ particle}$, we resample particles. Namely, we sample particles based on its weight. The greater a particle's weight is, the more likely it is to be selected. Usually, a particle with the largest weight tends to be sampled more than once. Then, we normalize each weight so that their sum is $1$. As for pre-set parameters, we set $w_{threshold} = \frac{1}{2 \cdot n_{particles}}$ and $n_{small\ particle} = \left \lfloor{\frac{n_{particles}}{2}}\right \rfloor$\\

(iv) defining time step  
We pick a range of timestamp $[1544582648.8, 1544583808.8)$ and divided this range by $1.0$ second and use $[t_{start}, t_{start}+1.0)$ as unit time step and all procedure such as prediction, updating weight, computing sample mean and covariance of velocities, updating occupancy map are done in the unit time step. For example, first we compute initial value of sample mean and covariance of velocities in time step $[1544582648.8, 1544582648.8+1)$ and then we iterate prediction and other procedures in the next step, $[1544582648.8+1, 1544582648.8+2)$ and then do the same in $[1544582648.8+2, 1544582648.8+3)$ and so on until it reaches the last step $[1544583808.8-1, 1544583808.8)$.

#### Occupancy Grid Map
We first adjust coordinate of each object in body frame at time $t$ computed from Lidar data to the coordiante of the world frame using the estimated location of car body at time $t$ by particle filter as follows  
<img src=
"https://user-images.githubusercontent.com/15256774/112426936-2455dc80-8d7c-11eb-9947-f6f2fbfef68c.png" 
alt="corr_{t, i} = \sum^m_{j=1}\mathbb{1}_{m_{i,j} = m_{global, j}}"  width="700"> 

where i = 1,2,... denotes each object of Lidar data.

Then, using these coordinates of objects in the world frame, we assign each object integer-valued coordinate of the grid map which is closest to the continuous-valued coordinate. Then, using bresenham2D function, we determine not-occupied cell and occupied cell. Finally, we compute log-odds of occupancy of each cells and accumulated it by simply adding +log4 or -log4 as follows
<img src=
"https://user-images.githubusercontent.com/15256774/112427261-a1815180-8d7c-11eb-8842-0d2dc7b8b3ad.png" 
alt="corr_{t, i} = \sum^m_{j=1}\mathbb{1}_{m_{i,j} = m_{global, j}}"  width="400">

In implementation, we use $\lambda_{min} = -5, \lambda_{max} = 5$ and we maintained all accumulated log-odds in $[\lambda_{min}, \lambda_{max}]$, and decay of log-odds $= 0.999$.  

#### Map Correlation
We compute correlation between a localized occupancy map based on estimated location by a particle at time $t+1$ and a global occupancy map estimated until time $t$ as follows  
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+corr_%7Bt%2C+i%7D+%3D+%5Csum%5Em_%7Bj%3D1%7D%5Cmathbb%7B1%7D_%7Bm_%7Bi%2Cj%7D+%3D+m_%7Bglobal%2C+j%7D%7D" 
alt="corr_{t, i} = \sum^m_{j=1}\mathbb{1}_{m_{i,j} = m_{global, j}}"  width="300">  
where $i, j$ denote each particle and each cell of occupancy map.
