# DopRIO: DopRIO: Doppler Strengthened Tightly-Coupled 4D Millimeter-Wave Radar–Inertial Odometry

## 1.Dependency
### 1.1 Ubuntu and Ros  
Ubuntu 20.04 and Ros Noetic
### 1.2 Required library
Eigen3, PCL, Sophus(fmt), Boost, OpenMP, TBB
### 1.3 Required ROS packages
pcl_ros, pcl_conversions, sensor_msgs, geometry_msgs, nav_msgs
## 2.Run the project
Download our project and put it into your workspace catkin

First you need to compile it

`catkin build`

`source devel/setup.bash`

Then you can run our launch file

`roslaunch doprio doprio_serial.launch`

## 3.Acknowlegement
[slam in autonomous driving](https://github.com/gaoxiang12/slam_in_autonomous_driving)
[slambook-en](https://github.com/gaoxiang12/slambook-en) , [slam in autonomous driving](https://github.com/gaoxiang12/slam_in_autonomous_driving) and [Dr. Gao Xiang](https://github.com/gaoxiang12). His SLAM tutorial and code are the starting point of our SLAM journey.

[Coloradar](https://arpg.github.io/coloradar/) and [NTU4DRADLM](https://github.com/junzhang2016/NTU4DRadLM). Many thanks to all the authors of the two public datasets.

## 4.Implementation Details
Detailed explanations and additional resources will be released upon the acceptance of our paper.

## 5.Demo
Demo in the Outdoors6 sequence of ColoRadar Dataset 
![Demo](apps/pic/ourdoors6.gif)
Demo in the 2nd sequence of Our collected real-world data 
![Demo](apps/pic/ours1.gif)
