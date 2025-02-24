# ros2_robolab
The robolab heretic repository



zed_components/src/zed_camera/src/zed_camera_component.cpp

-  transformStamped.child_frame_id = mBaseFrameId;
+  // transformStamped.child_frame_id = mBaseFrameId;
+  transformStamped.child_frame_id = "base_link";
