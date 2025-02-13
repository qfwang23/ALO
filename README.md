# ALO: An Adaptive LiDAR Odometry Approach for Dynamic Environments

![](https://github.com/qfwang23/ALO/blob/3097dbb8654d6bd3e535c46e0c620b0a3eef7c20/fig1.png)
## Catalog
- [Introduction](#Introduction)
- [Dependency](#Dependency)
- [Install](#Install)
- [Rusult](#Rusult)
- [Acknowledgements ](#Acknowledgements)
 
## Introduction
 
LiDAR captures environmental information to enable precise spatial localization and mapping. However, traditional LiDAR odometry methods mainly depend on static environmental features for positioning and mapping, limiting adaptability in dynamic settings and reducing pose estimation accuracy. To overcome this limitation, we propose ALO, a novel adaptive LiDAR odometry approach designed for dynamic environments. First, an adaptive constant velocity model predicts the expected motion trajectory, supplying prior pose information, while a first-in-first-out voxel grid manages the local map in dynamic conditions. Next, a linear system with dynamic weights based on point-surface residuals is established, minimizing the influence of dynamic features on pose estimation. Finally, the predicted prior pose serves as the initial value for adaptive ICP registration, enhancing pose estimation accuracy and enabling real-time local map updates.

## Dependency
```bash
Ubuntu 18.04 or 20.04
ROS Melodic（roscpp、std_msgs、sensor_msgs、geometry_msgs、pcl_ros）
C++ 14
CMake ≥ 3.16
PCL≥ 1.10.0
Eigen ≥ 3.3.7
```

## Install
 
```bash
git clone https://github.com/qfwang23/ALO.git
cd ALO
mkdir src
cd ..
catkin_make
source setup.bash
roslaunch ALO odometry.launch
rosbag play [topic]
```

## Rusult

![示例图片](https://github.com/qfwang23/ALO/blob/b20e13e008f5c0c39613f8e8ad7b2543110a44c5/fig3.png)

![示例图片](https://github.com/qfwang23/ALO/blob/a821eeba42d03649c9a34c21c116e969c9f4e7f8/fig4.png)

![示例图片](https://github.com/qfwang23/ALO/blob/a821eeba42d03649c9a34c21c116e969c9f4e7f8/fig5.png)

## Acknowledgements
```bash
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1029--1036},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023},
}

@article{chen2022direct,
  author={Kenny Chen and Brett T. Lopez and Ali-akbar Agha-mohammadi and Ankur Mehta},
  journal={IEEE Robotics and Automation Letters}, 
  title={Direct LiDAR Odometry: Fast Localization With Dense Point Clouds}, 
  year={2022},
  volume={7},
  number={2},
  pages={2000-2007},
  doi={10.1109/LRA.2022.3142739}
}
```

