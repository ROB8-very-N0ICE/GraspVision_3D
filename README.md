# GraspVision 3D
The project uses an Intel depth camera to determine the optimal grasp type for a robotic prosthetic hand to receive an object from another person's hand. The goal is to simplify the use of prosthetic hands and enhance embodiment.

1. Perform semantic segmentation to identify pixels belonging to the individual (including their hand). 
2. Generate a point cloud.
3. Eliminate points corresponding to pixels identified as the person.
4. Remove points that fall outside a specified distance threshold.
5. Employ 3D RANSAC to fit the object's shape and size, considering options like a sphere, cylinder, or cuboid.
6. Determine the appropriate grasp type and aperture based on the fitted shape and size.

Disclaimer: The project's focus is on decision-making and does not involve any form of direct control of a prosthesis.

<img title="Depth map object segmentation" alt="Alt text" src="/camera_tests.gif">

## Instalation
In a terminal, navigate to the folder where the project will be installed and type the following
```
# Clone this project repository
git clone https://github.com/ROB8-very-N0ICE/P8_project.git
cd P8_project/src

# Install camera dependencies
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/ros/dynamic_reconfigure.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

# Install segmentation CNN (Yolact)
git clone https://github.com/Eruvae/yolact_ros_msgs.git
git clone https://github.com/Eruvae/yolact_ros.git

# Include ROS independent yolact dependencies
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