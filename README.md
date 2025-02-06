# 项目名称
 
简短的描述，说明项目的用途和目标。
 
## 目录
 
- [简介](#简介)
- [安装](#安装)
- [使用方法](#使用方法)
- [贡献](#贡献)
- [许可证](#许可证)
 
## 简介
 
详细描述你的项目是什么，解决了什么问题，它的主要功能和优势。
 
## 安装
 
提供安装项目的步骤，例如：
 
```bash
git clone https://github.com/你的用户名/你的项目.git
cd 你的项目
npm install # 如果你使用Node.js
```

## 贡献
To address the insufficient adaptability in dynamic environments, we propose a method for calculating adaptive error thresholds based on velocity, acceleration, and motion differences. This approach dynamically adjusts the system's responsiveness to environmental changes, improving the stability and accuracy of pose estimation.

To reduce the impact of dynamic features on positioning accuracy, we develop a residual-based dynamically weighted linear system. Combined with the adaptive error threshold, this system performs adaptive scan-to-map registration, further enhancing pose estimation accuracy.

To validate the effectiveness of these methods in real-world scenarios, we conduct extensive experiments using the public KITTI dataset. The comparative results show that the proposed method outperforms existing mainstream LiDAR odometry methods in terms of accuracy. 
