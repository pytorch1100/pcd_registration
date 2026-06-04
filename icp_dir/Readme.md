# ICP求解

## 1.1-点云基础数据类型：1_icp_test.cpp

测试pcl库，CMakeLists.txt到main函数，头文件引用等。

### ROS点云类型



### PCL点云类型



## 1.2 点云IO 

+ 核心头文件：pcl/io/pcd_io.h   pcl/io/io.h
+ 常用类/函数：pcl::loadPCDFile、 savePCDFile、 rosToPCL
+ 重点学习目标：实现PCD文件读写，ROS和PCL点云类型转换



## 1.3 ICP点云配准

给定两坨点云

+ （1）两组点云已经匹配好，且数量相等，可以通过最小化下面的公式求解R和t
+ （2）实际情况下，点云之间的匹配关系未知，且点的数量不相等。因此，ICP采用迭代的方法求解
  + 迭代步骤如下：
  + step1：从两组点云中选择匹配的点对，形成匹配两组点云
  + step2：利用（1）中的方法求解R 和 t
  + step3：

### kd-tree寻找匹配点对



