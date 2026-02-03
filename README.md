
Launch controller

```
source install/setup.bash 
ros2 launch franka_moveit_compliant_control compliant_moveit.launch.py robot_ip:=172.18.0.2
```

Example trajectory to test the controller

```
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.45, y: 0.05, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.25, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}]}"
```