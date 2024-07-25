# Moveo-2
Setup and control of a BCN3D Moveo Robotic Arm using Moveit 2.

...More info on the Notion Page...

### Objectives:
- Implement a feedback loop using absolute position sensors, in an already built BCN3D Moveo Robotic Arm.
- Control it wirelessly using ROS2.

## Folders
### Meshes
Contains the STL files, correctly oriented and centered, some from the original BNC3D Moveo](https://www.thingiverse.com/thing:1693444), and some modified.
>[!NOTE]
>!Enumerate which ones were modified!

The installed end effector is the [mantis gripper](https://www.thingiverse.com/thing:1480408), and an [adapter](https://www.thingiverse.com/thing:1783754) was used to install it.
### Ros2 Packages:
Ros2 Packages used throughout the project.
Each iteration starts from the previous and takes the project one step closer to the finished product

#### -iteration1
Loads URDF into RViz and control joints using sliders
...More info On the Notion Page...

#### -iteration2
Motion planning on Rviz with MoveIt2
...More info On the Notion Page...

#### -iteration3
Integration with gazebo simulator
...More info On the Notion Page...

https://github.com/user-attachments/assets/3933dd15-cce5-4659-bdfb-399a65d297fb


#### -iteration4
UNDER DEVELOPMENT

#### -Arm_try_3 (Verify if it can be deleted)
Arm_try_3 is a MoveIT package, created using the MoveIt Setup Assistant. Some files were modified so the package would work correctly.
The URDF of the robotic arm can be found at arm_try_3/config/moveo_urdf.urdf


### Microcontroller
Arduino Firmware, final deploys but also hardware test sketches.


## Installation

### Ros2 Humble:

- Install using Debian packages (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Moveit:




- Install  (https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)
