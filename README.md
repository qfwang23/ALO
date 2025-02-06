# ALO: An Adaptive LiDAR Odometry Approach for Dynamic Environments

![Overview of ALO. The diagram includes adaptive motion prediction, local maps of  voxels, dynamic weight linear systems, adaptive ICP registration, and adaptive error threshold.](https://github.com/qfwang23/ALO/blob/3097dbb8654d6bd3e535c46e0c620b0a3eef7c20/fig1.png)
## Catalog
- [Introduction](#Introduction)
- [Install](#Install)
- [Rusult](#Rusult)
 
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

## Rusult

![示例图片](https://github.com/qfwang23/ALO/blob/b20e13e008f5c0c39613f8e8ad7b2543110a44c5/fig3.png)

![示例图片](https://github.com/qfwang23/ALO/blob/a821eeba42d03649c9a34c21c116e969c9f4e7f8/fig4.png)

![示例图片](https://github.com/qfwang23/ALO/blob/a821eeba42d03649c9a34c21c116e969c9f4e7f8/fig5.png)


