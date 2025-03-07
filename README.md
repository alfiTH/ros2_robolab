# ros2_robolab
The robolab heretic repository

You must clone the repository with 
```bash
git clone --recurse-submodules 
```

# Dependencies

## [Install ROS humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe


sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop colcon python3-colcon-common-extensions
pip install catkin_pkg
```

## Ros2bag y otras dependencias

```bash
sudo apt-get install ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-cv-bridge ros-humble-image-transport ros-humble-image-transport-plugins

```

# compilado
```bash
source /opt/ros/humble/setup.bash 
colcon build
source install/setup.bash 
```
si tienes fallo en la cámara 360 con al librería cv_bidge, cambia el .h por .hpp

# Ejecución
```bash 
source /opt/ros/humble/setup.bash 
source install/setup.bash 
rviz2
# Una vez abierto arriba file carga la congifuración de src/shadow/config/shadow_rviz_compressed.rviz
```

En otra terminal 
```bash 
source /opt/ros/humble/setup.bash 
source install/setup.bash 
ros2 bag play ../rosbag2_2025_03_06-17_58_18/ --loop --clock #--rate 1.0
```

En otra terminal 
```bash 
source /opt/ros/humble/setup.bash 
source install/setup.bash 
ros2 run anonymizer anonymizer_node 
```


# Same changes

## zed-ros2-wrapper
- modified:   zed_components/src/zed_camera/src/zed_camera_component.cpp
- modified:   zed_wrapper/config/common_stereo.yaml
- modified:   zed_wrapper/config/zed2i.yaml
- modified:   zed_wrapper/launch/zed_camera.launch.py

### zed_components/src/zed_camera/src/zed_camera_component.cpp
Change line 6610
```c++
    transformStamped.child_frame_id = mBaseFrameId;
```
to 
```c++
    // transformStamped.child_frame_id = mBaseFrameId;
    transformStamped.child_frame_id = "base_link";
```

### zed_wrapper/config/common_stereo.yaml
```bash
cp configs/common_stereo.yaml src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml
```

### zed_wrapper/config/zed2i.yaml
```bash
cp configs/zed2i.yaml src/zed-ros2-wrapper/zed_wrapper/config/zed2i.yaml
```
### zed_wrapper/launch/zed_camera.launch.py
Change line 301
```python
             DeclareLaunchArgument(
                 'publish_map_tf',
-                default_value='true',
```
to
```python
             DeclareLaunchArgument(
                 'publish_map_tf',
                default_value='false',
```