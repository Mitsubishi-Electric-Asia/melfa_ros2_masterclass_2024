<img src="./figures/MELFA_t.png" width="400" height="98">

# __MELFA ROS2 & MELSOFT Simulator Setup__

This menu provides a guide to setup MELFA ROS2 Driver on ROS2 Humble, Ubuntu 22.04LTS and associated MELSOFT simulators in Windows10.

## __1. MELFA ROS2 Setup__

This section provides a guide to setup MELFA ROS2 Driver and MELFA ROS2 Masterclass 2024 on your __Ubuntu 22.04LTS device__.

#### 1. Create your MELFA workspace
```
mkdir -p ~/melfa_ws/src
cd ~/melfa_ws/src
```
#### 2. Download MELFA ROS2 Masterclass 2024 [Version 1.0.0](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_masterclass_2024/archive/refs/tags/v1.0.0.zip) from github. Delete "MELSOFT Project Files.zip" as this is for the Windows10 device.

Cloning is not recommended as the latest version of MELFA ROS2 Masterclass 2024 may have compatibility issues with the latest version of MELFA ROS2 Driver.

#### 3. Download MELFA ROS2 Driver [Version 1.0.3](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_driver/archive/refs/tags/v1.0.3.zip) from github. 

Cloning is not recommended as the latest version of MELFA ROS2 Driver may have compatibility issues with the latest version of MELFA ROS2 Masterclass 2024.

#### 4. Extract MELFA ROS2 Driver to melfa_ws/src.
Ensure that melfa_ros2_driver-1.0.3 and this version of melfa_ros2_masterclass_2024 are in melfa_ws/src.

#### 5. Update package dependency sources
```
cd ~/melfa_ws
rosdep update
```
#### 6. Install Install dependencies
```
cd ~/melfa_ws
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y --skip-keys warehouse_ros_mongo
```
#### 7. Open a terminal and build melfa_ws.
```
cd ~/melfa_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Your MELFA workspace is ready!

## 2. MELSOFT Simulator Setup

This section provides a guide to setup MELFA ROS2 Masterclass 2024 on your __Windows10 device__.

#### 1. Download MELFA ROS2 Masterclass 2024 [Version 1.0.0](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_masterclass_2024/archive/refs/tags/v1.0.0.zip) from github. 

The repository contains files for MELFA ROS2 and MELSOFT. 

#### 2. Extract the files from "MELSOFT Project Files.zip".

