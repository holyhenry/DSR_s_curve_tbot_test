# Decentralized TurtleBot3 formation calibration (ROS Noetic)

Each follower runs **one independent instance** of `formation_calibration.py` on its own TurtleBot3 computer. It subscribes **only to its own** `geometry_msgs/PoseStamped` VRPN topic and publishes **only to its own** velocity topic. It does not wait for, coordinate, or command another follower. The robots may run the same launch file at different times. This is one-shot initial positioning, not decentralized collision avoidance or formation tracking.

## Install / setup

Copy `tbot_launch/scripts/formation_calibration.py` and `tbot_launch/launch/formation_calibration.launch` to **each robot's** copy of the workspace; replace `tbot_launch/package.xml` and retain existing `CMakeLists.txt` (the previous patch already installs the script). If applying the complete patch, use its CMakeLists.txt too. On each TurtleBot3:

```bash
chmod +x ~/catkin_ws/src/DSR_s_curve_tbot_test/tbot_launch/scripts/formation_calibration.py
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

ROS master, TurtleBot3 base bringup, and VRPN bridge must be available. **Do not run `system_bringup.launch` / tracking controller while calibrating**, as that may create multiple `cmd_vel` publishers. Confirm actual cmd topic and robot physical identity first.

## Identity mapping

| follower_id | Robot | VRPN topic | cmd topic example |
| --- | --- | --- | --- |
| 1 | bot162 | `/vrpn_client_node/bot162/pose` | `/tbot162/cmd_vel` |
| 2 | bot149 | `/vrpn_client_node/bot149/pose` | `/tbot149/cmd_vel` |
| 3 | bot165 | `/vrpn_client_node/bot165/pose` | `/tbot165/cmd_vel` |

`cmd_topic` defaults to a namespace selected from `follower_id`. Always verify it and preferably pass `cmd_topic` explicitly for every robot. The `/tbot149` and `/tbot165` topic names shown here are **assumptions**; verify them using `rostopic list` / `rostopic info` before live operation. The node defaults to a failing `follower_id=0` if not provided. Valid IDs: 1–3 in platoon, 1–2 in V-formation.

## Targets

Every robot computes its **own** fixed formation goal using the **same** leader reference pose and formation settings. For the simulation-based defaults, platoon leader is `(-0.5, 0.8, -pi)` and V leader is `(2.5, 0.8, -pi)`; these need verification in your physical experiment. Platoon: follower `i` has body offset `(-i*spacing,0)`. V: follower 1 has `(-arrow_a,+arrow_b)`; follower 2 `(-arrow_a,-arrow_b)`. Both modes rotate the offset into the global mocap frame using the leader reference yaw. These formulas must match the desired experiment start poses; the simulation platoon initializer should be checked separately if strict equivalence is required.

## Commands: each run on the specified robot's own terminal

Check mocap with `rostopic hz /vrpn_client_node/bot162/pose`, etc. Start in dry-run mode (only zero commands, does **not** complete unless manually positioned):

```bash
# ON bot162
roslaunch tbot_launch formation_calibration.launch follower_id:=1 formation_type:=platoon dry_run:=true cmd_topic:=/tbot162/cmd_vel

# ON bot149
roslaunch tbot_launch formation_calibration.launch follower_id:=2 formation_type:=platoon dry_run:=true cmd_topic:=/tbot149/cmd_vel

# ON bot165
roslaunch tbot_launch formation_calibration.launch follower_id:=3 formation_type:=platoon dry_run:=true cmd_topic:=/tbot165/cmd_vel
```

After verifying targets, pose frame, command topics and an unobstructed path, run **one robot at a time** at reduced speed first:

```bash
# ON bot162 only
roslaunch tbot_launch formation_calibration.launch follower_id:=1 formation_type:=platoon dry_run:=false max_linear_speed:=0.03 cmd_topic:=/tbot162/cmd_vel
```

For V-formation set `formation_type:=v_formation` on bot162 and bot149, with IDs 1 and 2. Do not run calibration on bot165 in V mode.

The tested practical tolerance is **0.02 m** and heading tolerance **3 degrees**. Successful convergence is maintained over a 0.5 s settling period, followed by repeated zero commands and automatic exit. Each robot can finish separately. The launch uses `required=false`, avoiding misleading “required process died” messaging after normal completion. The ROS launch command might remain running if other nodes are included, but this calibration node exits cleanly.

## Safety and limitations

- One robot commands only its own absolute `cmd_topic`; misconfiguring that parameter could still command the wrong physical robot. Inspect `rostopic info` and verify robot namespace identity.
- There is **no inter-robot collision avoidance or synchronizing barrier** in this decentralized version. Do not start multiple robots from potentially conflicting paths unless external coordination and collision safeguards are provided. Start sequentially for commissioning.
- The code assumes VRPN reports the TurtleBot3 base center and base-forward yaw in the **same** global frame as configured leader pose. Verify or apply an external rigid body to base transform.
- Mocap timestamp and reception freshness are enforced. Robot stops on stale observations, timeout, normal exit and shutdown, with zero Twist publications. Use a functioning base watchdog and accessible E-stop. The configured timeout is 120 s.
- The launch file does not include base bringup; start it separately, without the tracking publisher. After successful calibration, ensure no calibration node remains publishing before starting your regular tracking launch.

## Static checks without ROS

```bash
python3 -m unittest discover -s tbot_launch/test -p 'test_formation_calibration.py'
python3 -m py_compile tbot_launch/scripts/formation_calibration.py
```

## Mocap frame to simulation/experiment frame (tf.pdf)

The controller now converts **each received VRPN pose** before calculating
position/bearing errors. The leader reference and formation targets remain
expressed in the simulation/experiment frame.

The drawn mocap +x axis points up, +y points left, and its origin is
(-0.355, -0.722) in the experiment frame. Therefore
`T_experiment_mocap = Trans(-0.355, -0.722) @ Rot(+90 deg)`; for a VRPN pose
`(x_m, y_m, yaw_m)`, the transformed values are
`x_e=-y_m-0.355`, `y_e=x_m-0.722`, `yaw_e=wrap(yaw_m+pi/2)`.
The user-described clockwise rotation refers to frame axes; the numerical
coordinate conversion is +90 degrees. The rotation can be overridden with
`tf_rotation_deg` if frame alignment is different.

All three parameters are launch arguments: `tf_horizontal_displacement:=0.355`,
`tf_vertical_displacement:=0.722`, `tf_rotation_deg:=90.0`. The displacements
are **positive measured distances** but are subtracted in experiment coordinates.
Run `dry_run:=true` to confirm transformed position and target before live motion.
The transform assumes the VRPN rigid-body axes correspond to the drawn mocap
axes and that there is no additional marker-to-base yaw/position offset.
