# P8_project
Prosthesis perception of human-robot interaction for amputee social engagement

## Instalation
Install our system
```
git clone https://github.com/ROB8-very-N0ICE/P8_project.git
cd P8_project/src
```
Install camera dependencies
```
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/ros/dynamic_reconfigure.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
```
Install segmentation CNN (Yolact)
```
git clone https://github.com/dbolya/yolact.git
git clone https://github.com/Eruvae/yolact_ros_msgs
git clone https://github.com/Eruvae/yolact_ros.git
```

## Usage
Launch camera
```
roslaunch prosthetic_hand camera.launch
```
