# vive_ros2 (development in progress)

## Development Status
- [x]  Obtain absolute pose data of controller
- [x]  Implement the server client model
- [x]  Haptic feedback to enhance user experience
- [x]  Solve relative transformations
- [x]  Add bounding conditions 
- [ ]  Optimize performance

## Overview

This `vive_ros2` package provides a ROS2 interface to the HTC VIVE controllers. Due to compatibility issues between the OpenVR library and ROS2, this package utilizes socket programming to enable data transfer between two standalone programs running under ROS2 on Ubuntu.

## Installation instructions

### 1. Install Steam and SteamVR
1. Install the latest version of Steam from [Steam Store](https://store.steampowered.com/).
2. Install SteamVR in the Steam application.

### 2. Downlaad and Build OpenVR SDK
```bash
cd <PATH to ROS2 workspace>
mkdir libraries && cd libraries
git clone https://github.com/ValveSoftware/openvr.git -b v2.5.1
cd openvr
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

### 3. VR Setup
1. Follow the official [VIVE Pro Setup Guide](https://www.vive.com/hk/setup/vive-pro-hmd/) to setup the lighthouse, headset, controller, etc.
2. Plug in the VIVE and set the permission as below:
    ```bash
    sudo chmod +rw /dev/hidraw*
    ```
3. You are ready to use the VIVE.
4. (OPTIONAL) Build the sample code to test the VIVE setup.
    ```bash
    cd <PATH to ROS2 workspace>/libraries/openvr/samples
    mkdir build && cd build
    cmake .. -G "Unix Makefiles" -DCMAKE_PREFIX_PATH=/opt/Qt/5.6/gcc_64/lib/cmake -DCMAKE_BUILD_TYPE=Release
    ```
    Run the demo code as follows:
    ```bash
    # copy the texture files to the bin folder
    cd <PATH to ROS2 workspace>
    cp libraries/openvr/samples/bin/cube_texture.png libraries/openvr/samples/bin/hellovr_* build
    ```
    ```bash
    # Run demo code
    ~/.steam/steam/ubuntu12_32/steam-runtime/run.sh <PATH to ROS2 workspace>/libraries/openvr/samples/bin/linux64/hellovr_opengl
    ```
    

## Usage
1. Build the package.
    ```bash
    cd <PATH to ROS2 workspace>
    colcon build --packages-select vive_ros2
    source install/setup.bash
    ```
2. Start SteamVR.
    ```bash
    source <PATH to ROS2 workspace>/src/vive_ros2/scripts/set_vr_env.sh
    $STEAMVR/bin/linux64/vrserver --keepalive
    ```
3. Run the package.
    ```bash
    # Terminal 1:
    ros2 run vive_ros2 vive_input
    # Terminal 2:
    ros2 run vive_ros2 vive_node
    ```

## Demo
Using VIVE Pro controller to control a [WidowX-250-S](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html) robot arm in ROS2 (using absolute pose).
![VIVE Pro Demo](docs/videos/vive_pose-abs-control.gif)

Visualizing the absolute and relative poses of the controller on RViz.
![VIVE Pro Demo](docs/videos/vive_pose-relative.gif)

## Development Environment
- Ubuntu 22.04
- ROS2 Humble
- OpenVR SDK v2.5.1
- HTC VIVE Pro
