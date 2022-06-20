# P8_project
Prosthesis perception of human-robot interaction for amputee social engagement

<img title="Depth map object segmentation" alt="Alt text" src="/camera_tests.gif">

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
git clone https://github.com/Eruvae/yolact_ros_msgs.git
git clone https://github.com/Eruvae/yolact_ros.git
cd ../yolact_ros/scripts
git clone https://github.com/dbolya/yolact.git
mkdir yolact/weights
cd yolact/weights
```
Download the models into the folder weights.
Add `#!/usr/bin/env python3` to the first line of yolact python scripts.


## Usage
Launch camera
```
roslaunch prosthetic_hand camera.launch
```

to catkin_make with the cv_bridge... I tried
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
```
but then
