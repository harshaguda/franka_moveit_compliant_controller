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
ros2 topic pub --once /cartesian_trajectory geometry_msgs/msg/PoseArray "{header: {frame_id: 'fr3_link0'}, poses: [{position: {x: 0.45, y: -0.15, z: 0.35}, orientation: {x: -0.7071, y: 0.7071, z: 0.0, w: 0.0}}]}"
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

Use Bolt planner for constrained planning (e.g. dressing):

```
ros2 launch franka_moveit_compliant_control compliant_moveit.launch.py robot_ip:=<robot_ip> lock_orientation:=true planner_id:=BoltConfigDefault
```

Use Descartes planning pipeline (requires descartes capability to be configured):

```
ros2 launch franka_moveit_compliant_control compliant_moveit.launch.py robot_ip:=<robot_ip> planning_pipeline:=descartes
```

Attention-aware interruption/pause during execution:

- The executor now listens to `/engagement/status` (configurable via `attention_status_topic`).
- If `pause_when_not_attention:=true` (default), an active MoveIt trajectory is interrupted with `stop()` as soon as attention is lost.
- Execution resumes automatically once attention is back by replanning from the robot's current pose to the pending waypoint.

Useful parameters (set on `cartesian_trajectory_executor`):

- `attention_status_topic` (default: `/engagement/status`)
- `pause_when_not_attention` (default: `true`)
- `attention_ok_values` (default: `['1', 'true', 'paying_attention']`)
- `attention_poll_ms` (default: `50`)