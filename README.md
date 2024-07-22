# vive_ros2 (development in progress)

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
    ros2 run vive_ros2 vive_input
    # Development in Progress...
    ```


## Development Environment
- Ubuntu 22.04
- ROS2 Humble
- OpenVR SDK v2.5.1
