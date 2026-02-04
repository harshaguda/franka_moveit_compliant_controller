
Launch controller

```
source install/setup.bash 
ros2 launch franka_moveit_compliant_control compliant_moveit.launch.py robot_ip:=172.18.0.2
```

Example trajectory to test the controller

```
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.45, y: 0.05, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.25, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}]}"
```

Take the robot to initial dressing position

```
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.45, y: 0.045, z: 0.35}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}]}"
```

Dressing Start

```
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}]}"

```

```
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.4, y: 0.70, z: 0.3}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}]}"
```

```
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.45, y: 0.045, z: 0.35}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.3089, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.4089, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.5089, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.6089, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.7089, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.8089, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}, {position: {x: 0.45, y: 0.9, z: 0.2693}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}]}"
```


```
pkill -f joint_state_publisher || true
ros2 daemon stop && ros2 daemon start
```