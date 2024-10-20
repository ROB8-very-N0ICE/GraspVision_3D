
# GraspVision 3D

GraspVision 3D is a project that utilizes an Intel depth camera to determine the optimal grasp type for a robotic prosthetic hand to receive an object from another person's hand. The project's primary aim is to simplify the use of prosthetic hands and improve the sense of embodiment for users by automating the grasping process.

### Project Workflow:
1. **Semantic Segmentation**: Perform semantic segmentation to identify pixels belonging to the individual (including their hand).
2. **Point Cloud Generation**: Generate a point cloud based on depth data.
3. **Point Filtering**: Remove points corresponding to pixels identified as the person.
4. **Distance Thresholding**: Discard points outside a predefined distance range.
5. **Shape Fitting**: Use 3D RANSAC to fit the object's shape and size, selecting from options like spheres, cylinders, or cuboids.
6. **Grasp Determination**: Choose the appropriate grasp type and aperture based on the fitted object's shape and size.

> **Disclaimer**: The project focuses solely on decision-making and does not involve direct control of the prosthesis.

![Depth map object segmentation](/camera_tests.gif "Depth map object segmentation")

---

## System Requirements
- **OS**: Ubuntu 18.04
- **ROS Version**: Foxy
- **Camera**: Intel RealSense D435

---

## Installation Instructions

1. Open a terminal and navigate to the directory where you want to install the project.

2. Clone the project repository and move into the source directory:
   ```bash
   git clone https://github.com/ROB8-very-N0ICE/P8_project.git
   cd P8_project/src
   ```

3. Install the camera dependencies:
   ```bash
   git clone https://github.com/IntelRealSense/realsense-ros.git
   git clone https://github.com/ros/dynamic_reconfigure.git
   git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
   ```

4. Install the segmentation CNN (Yolact):
   ```bash
   git clone https://github.com/Eruvae/yolact_ros_msgs.git
   git clone https://github.com/Eruvae/yolact_ros.git
   ```

5. Download the Yolact models into the `weights` folder.

6. Modify Yolact scripts to ensure Python 3 compatibility by adding the following to the first line of each Python script:
   ```bash
   #!/usr/bin/env python3
   ```

7. Include ROS-independent Yolact dependencies:
   ```bash
   cd ../yolact_ros/scripts
   git clone https://github.com/dbolya/yolact.git
   mkdir yolact/weights
   cd yolact/weights
   ```

---

## Usage

1. To launch the camera, run the following:
   ```bash
   roslaunch prosthetic_hand camera.launch
   ```

2. To build the workspace with `cv_bridge` dependencies, use the following `catkin_make` command:
   ```bash
   catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
   ```

---

## Notes
- Ensure all dependencies are properly installed before launching the project.
- Python 3 compatibility may require adjustments in some scripts.
- The project focuses on grasp type decision-making and does not include control over a physical prosthesis.

---

## License
This project is open-source and licensed under the MIT License. Feel free to contribute and adapt the code for your use cases.