# ros2_robolab
The robolab heretic repository





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
cp config/common_stereo.yaml src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml
```

### zed_wrapper/config/zed2i.yaml
```bash
cp config/zed2i.yaml src/zed-ros2-wrapper/zed_wrapper/config/zed2i.yaml
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
