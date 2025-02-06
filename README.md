# 项目名称
 
简短的描述，说明项目的用途和目标。
 
## 目录
 
- [Introduction](#Introduction)
- [Install](#Install)
- [使用方法](#使用方法)
- [贡献](#贡献)
- [许可证](#许可证)
 
## Introduction
 
LiDAR captures environmental information to enable precise spatial localization and mapping. However, traditional LiDAR odometry methods mainly depend on static environmental features for positioning and mapping, limiting adaptability in dynamic settings and reducing pose estimation accuracy. To overcome this limitation, we propose ALO, a novel adaptive LiDAR odometry approach designed for dynamic environments. First, an adaptive constant velocity model predicts the expected motion trajectory, supplying prior pose information, while a first-in-first-out voxel grid manages the local map in dynamic conditions. Next, a linear system with dynamic weights based on point-surface residuals is established, minimizing the influence of dynamic features on pose estimation. Finally, the predicted prior pose serves as the initial value for adaptive Iterative Closest Point (ICP) registration, enhancing pose estimation accuracy and enabling real-time local map updates.
 
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

## 贡献
