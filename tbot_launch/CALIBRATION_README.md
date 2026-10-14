# TurtleBot3 formation calibration

Runs one centralized, one-shot `rospy` node to move 3 platoon or 2 V-formation followers to their global mocap-frame poses. This node must be launched **without** the existing DSR tracking publishers running.

## Important prerequisites

1. Start the ROS master, VRPN mocap bridge, and the **robot base bringups only**. `system_bringup.launch` is *not* appropriate during calibration because it starts `tracking_node`. Do not start multiple base bringups on the same robot.
2. Confirm `/vrpn_client_node/bot162` (follower 1), `/vrpn_client_node/bot149` (follower 2), and `/vrpn_client_node/bot165` (follower 3) are `geometry_msgs/PoseStamped` and return correct, recent, same-frame poses. For V-formation, follower 3 is ignored.
3. Confirm each follower's actual absolute `cmd_vel` topic. The example `/tb3_1/cmd_vel`, `/tb3_2/cmd_vel`, `/tb3_3/cmd_vel` names are **placeholders**, not verified against your robots. Set `f1_cmd`, `f2_cmd`, `f3_cmd` arguments appropriately.
4. Confirm the mocap rigid-body origin and orientation coincide with the TurtleBot3 `base_link` pose. A sensor-to-base transform must be applied upstream if they differ. The current node does not apply TF transformations.
5. Ensure the leader is stationary at `leader_x, leader_y, leader_yaw` for this initialization. The code uses this configured pose rather than subscribing to a leader topic. Keep the area clear. Initial separation and eventual paths must be visually checked; simple distance checks are not collision-proof planning.

## Calibration

First run a read-only dry run (only zero velocity is published):

```bash
roslaunch tbot_launch formation_calibration.launch formation_type:=platoon \
  f1_cmd:=/ACTUAL_F1_NAMESPACE/cmd_vel f2_cmd:=/ACTUAL_F2_NAMESPACE/cmd_vel \
  f3_cmd:=/ACTUAL_F3_NAMESPACE/cmd_vel
```

Then test **one follower at a time**, with emergency stop access, before enabling multi-robot movement. For physical calibration:

```bash
roslaunch tbot_launch formation_calibration.launch formation_type:=platoon \
  dry_run:=false f1_cmd:=/ACTUAL_F1_NAMESPACE/cmd_vel \
  f2_cmd:=/ACTUAL_F2_NAMESPACE/cmd_vel f3_cmd:=/ACTUAL_F3_NAMESPACE/cmd_vel
```

V-formation mode:

```bash
roslaunch tbot_launch formation_calibration.launch formation_type:=v_formation \
  dry_run:=false f1_cmd:=/ACTUAL_F1_NAMESPACE/cmd_vel f2_cmd:=/ACTUAL_F2_NAMESPACE/cmd_vel
```

The launch selects the following **simulation-specific** reference leader poses by default: `(-0.5, 0.8, -pi)` for platoon (platoon scenario 4) and `(2.5, 0.8, -pi)` for V-formation. These are NOT measured experimental targets. Override `leader_x`, `leader_y`, `leader_yaw` as appropriate. Platoon targets follow nominal 0.25 m spacing, which is a geometry assumption and should be checked against the real experiment.

`motion_mode:=sequential` is the default; `motion_mode:=simultaneous` is available after safety evaluation. Tolerances, gains, mocap timeout, 120-second overall deadline, and settling time are all launch parameters. Speed cannot be configured over 0.08 m/s. The script checks current pairwise positions and optionally the configured leader reference (`check_leader_clearance` parameter, default true). It does not perform path planning or predict collision along each path.

The node quits after all active robots hold their target pose within 5 mm and 3 degrees for the settling interval. Any failure, stale mocap observation, or ROS shutdown results in zero commands; on timeout or invalid configuration, it exits with failure. TurtleBot3 base watchdog and a physical emergency-stop procedure are still essential.

## Handoff to tracking

Once calibration exits successfully and no calibration node remains publishing velocity commands, start `system_bringup.launch` **on the appropriate robots**, avoiding duplicate base bringups. Bringup sequencing needs to match your existing multi-machine system. This node does not automatically start the leader navigation experiment.

## Static checks (without ROS installed)

```bash
python3 -m unittest discover -s tbot_launch/test -p 'test_*.py'
python3 -m py_compile tbot_launch/scripts/formation_calibration.py
```
