# Takeshi RC2025 simulation launch

[`takeshi_launch.launch.py`](takeshi_launch.launch.py) is the simulation entry
point. It includes three launch files in the same ROS 2 graph.

| Launch file | What it starts |
| --- | --- |
| [`takeshi_rc2025.launch.py`](takeshi_rc2025.launch.py) | The RC2025 Gazebo world from Robotino's `rto_description` package, Toyota HSR simulation and controllers, navigation, and RViz. |
| [`hsrb_moveit.launch.py`](hsrb_moveit.launch.py) | Toyota HSR MoveIt 2 with simulation time, plus the point-cloud republisher for OctoMap. |
| [`vision_takeshi.launch.py`](vision_takeshi.launch.py) | The vision nodes and known-locations server, which uses `~/.takeshi/rc_2025_known_locations.yaml`. |

The launch starts the components together. The test tree starts separately by
default, so you can wait for navigation, MoveIt, and the controllers to come up
before it sends motion goals.

## Terminal 1 — Start the simulation stack

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/takeshi_ws/install/setup.bash
ros2 launch takeshi_bringup takeshi_launch.launch.py
```

## Terminal 2 — Run the test tree

Use the same `ROS_DOMAIN_ID` and DDS settings in both terminals. Make sure
`~/.takeshi/rc_2025_known_locations.yaml` contains a `living_room` entry for the
RC2025 map.

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/takeshi_ws/install/setup.bash

ros2 control list_controllers
ros2 action list
```

Confirm `arm_trajectory_controller` and `head_trajectory_controller` are
`active` and that `/move_action` and `/move_base/move` are listed. Then run:

```bash
ros2 run takeshi_tree task_manager --ros-args -p use_sim_time:=true
```

The tree moves the arm to `go`, navigates to `living_room`, and moves the head
to pan `0.0` and tilt `-0.2` radians. Place a visible cup at that waypoint
for Robotino YOLO to detect. If multiple cups have valid 3D poses, the tree
selects the one closest to the camera. It uses TF at the detection timestamp to
compute a top-down palm pose 0.20 m above the object in `odom`, then executes
the approach with MoveIt's `arm` group. It stops at pregrasp.

The combined launch also accepts `start_tree:=true` to start the tree
concurrently. Its `use_sim_time` argument defaults to `true` and applies to
that optional tree node.

## Introspection

The combined launch opens the `py-trees-tree-viewer` GUI by default, using the
same launch setup as Robotino BTS. Start `task_manager` from Terminal 2 to see
the tree in the viewer. Use `with_viewer:=false` for a headless launch.

The tree publishes behavior status and the blackboard values used
by the active behaviors on `/tree/snapshots`. In a third terminal with the same
overlays and ROS domain, watch that topic before starting the tree.

```bash
ros2 topic echo /tree/snapshots --qos-durability transient_local
```
