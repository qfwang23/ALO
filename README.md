# 项目名称
 
简短的描述，说明项目的用途和目标。
 
## 目录
 
- [简介](#简介)
- [安装](#安装)
- [使用方法](#使用方法)
- [贡献](#贡献)
- [许可证](#许可证)
 
## 简介
 
LiDAR captures environmental information to enable precise spatial localization and mapping. However, traditional LiDAR odometry methods mainly depend on static environmental features for positioning and mapping, limiting adaptability in dynamic settings and reducing pose estimation accuracy. To overcome this limitation, we propose ALO, a novel adaptive LiDAR odometry approach designed for dynamic environments. First, an adaptive constant velocity model predicts the expected motion trajectory, supplying prior pose information, while a first-in-first-out voxel grid manages the local map in dynamic conditions. Next, a linear system with dynamic weights based on point-surface residuals is established, minimizing the influence of dynamic features on pose estimation. Finally, the predicted prior pose serves as the initial value for adaptive Iterative Closest Point (ICP) registration, enhancing pose estimation accuracy and enabling real-time local map updates.
 
## 安装
 
提供安装项目的步骤，例如：
 
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
To address the insufficient adaptability in dynamic environments, we propose a method for calculating adaptive error thresholds based on velocity, acceleration, and motion differences. This approach dynamically adjusts the system's responsiveness to environmental changes, improving the stability and accuracy of pose estimation.

To reduce the impact of dynamic features on positioning accuracy, we develop a residual-based dynamically weighted linear system. Combined with the adaptive error threshold, this system performs adaptive scan-to-map registration, further enhancing pose estimation accuracy.

To validate the effectiveness of these methods in real-world scenarios, we conduct extensive experiments using the public KITTI dataset. The comparative results show that the proposed method outperforms existing mainstream LiDAR odometry methods in terms of accuracy. 
