#!/usr/bin/env python3
"""Decentralized one-shot TurtleBot3 follower positioning (ROS Noetic)."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid mocap quaternion')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2*(w*z + x*y), 1-2*(y*y+z*z))


def formation_targets(mode, leader, spacing, arrow_a, arrow_b):
    x, y, yaw = leader
    if mode == 'platoon':
        offsets = [(-spacing*i, 0.0) for i in (1, 2, 3)]
    elif mode == 'v_formation':
        offsets = [(-arrow_a, arrow_b), (-arrow_a, -arrow_b)]
    else:
        raise ValueError('formation_type must be platoon or v_formation')
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*dx-s*dy, y+s*dx+c*dy, wrap_angle(yaw)) for dx, dy in offsets]


def staging_target(final_target, spacing):
    """One additional spacing behind the assigned final slot."""
    x, y, yaw = final_target
    return (x - spacing*math.cos(yaw), y - spacing*math.sin(yaw), yaw)


def mocap_to_experiment_pose(pose, horizontal_displacement, vertical_displacement,
                             rotation_deg=90.0):
    """Convert (x,y,yaw) from VRPN axes to experiment/simulation axes.

    In tf.pdf, mocap +x points up, mocap +y points left, and the mocap
    origin is (-horizontal_displacement, -vertical_displacement) in the
    experiment frame. Coordinates therefore use +90 deg (passive change
    of axes), despite the clockwise frame alignment description.

    T_experiment_mocap = Trans(-horizontal,-vertical) @ Rot(rotation_deg).
    """
    x, y, yaw = pose
    angle = math.radians(rotation_deg)
    c, s = math.cos(angle), math.sin(angle)
    return (c*x - s*y - horizontal_displacement,
            s*x + c*y - vertical_displacement,
            wrap_angle(yaw + angle))


class Robot:
    def __init__(self, index, pose_topic, cmd_topic, target):
        self.index = index
        self.pose_topic = pose_topic
        self.cmd_topic = cmd_topic
        self.target = target  # final formation slot
        self.staging_target = None
        self.pose = None
        self.last_received = None
        self.frame_id = None
        self.state = 'WAIT_FOR_MOCAP'
        self.settling_since = None
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback, queue_size=1)

    def _pose_callback(self, msg):
        try:
            p = msg.pose.position
            yaw = yaw_from_quaternion(msg.pose.orientation)
            if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                raise ValueError('Non-finite pose')
            # Reject old stamped observations; reception age is checked separately.
            if msg.header.stamp != rospy.Time(0):
                age = (rospy.Time.now() - msg.header.stamp).to_sec()
                if age < -0.2 or age > rospy.get_param('~mocap_timeout', 0.5):
                    return
            with self.lock:
                self.pose = (p.x, p.y, yaw)
                self.frame_id = msg.header.frame_id
                self.last_received = time.monotonic()
        except (ValueError, OverflowError) as exc:
            rospy.logwarn_throttle(2, 'Follower %d: %s', self.index, exc)

    def snapshot(self):
        with self.lock:
            return self.pose, self.last_received, self.frame_id

    def command(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)


class FormationCalibration:
    """One follower per process. No remote velocity publishers or all-robot barrier."""
    def __init__(self):
        def get(name, default):
            return rospy.get_param('~' + name, default)

        self.mode = get('formation_type', 'platoon')
        self.follower_id = int(get('follower_id', 0))
        self.dry_run = bool(get('dry_run', True))
        self.distance_tol = float(get('position_tolerance', 0.02))
        self.yaw_tol = math.radians(float(get('yaw_tolerance_deg', 3.0)))
        self.max_v = float(get('max_linear_speed', 0.08))
        self.max_w = float(get('max_angular_speed', 0.5))
        self.kp_v = float(get('kp_linear', 0.8))
        self.kp_w = float(get('kp_angular', 1.5))
        self.rate_hz = float(get('control_rate', 20.0))
        self.timeout = float(get('calibration_timeout', 120.0))
        self.mocap_timeout = float(get('mocap_timeout', 0.5))
        self.settle_time = float(get('settle_time', 0.5))
        self.turn_threshold = math.radians(float(get('turn_in_place_deg', 30.0)))
        self.tf_horizontal = float(get('tf_horizontal_displacement', 0.355))
        self.tf_vertical = float(get('tf_vertical_displacement', 0.722))
        self.tf_rotation_deg = float(get('tf_rotation_deg', 90.0))
        self.leader = (float(get('leader_x', -0.5)), float(get('leader_y', 0.8)),
                       float(get('leader_yaw', -math.pi)))
        spacing = float(get('spacing', 0.25))
        arrow_a = float(get('arrow_a', 0.25))
        arrow_b = float(get('arrow_b', 0.15))
        if not all(math.isfinite(n) for n in (self.tf_horizontal, self.tf_vertical, self.tf_rotation_deg)):
            raise ValueError('Frame-transform parameters must be finite')
        numbers = [self.distance_tol, self.yaw_tol, self.max_v, self.max_w,
                   self.kp_v, self.kp_w, self.rate_hz, self.timeout,
                   self.mocap_timeout, self.settle_time, spacing, arrow_a, arrow_b]
        if not all(math.isfinite(n) and n > 0 for n in numbers):
            raise ValueError('Positive finite gains, limits, spacings and thresholds required')
        if self.max_v > 0.08:
            raise ValueError('max_linear_speed must not exceed 0.08 m/s')
        targets = formation_targets(self.mode, self.leader, spacing, arrow_a, arrow_b)
        if not 1 <= self.follower_id <= len(targets):
            raise ValueError('follower_id must be 1..%d for %s' % (len(targets), self.mode))
        pose_topic = get('pose_topic', '')
        cmd_topic = get('cmd_topic', '/cmd_vel')
        if not pose_topic.startswith('/') or not cmd_topic.startswith('/'):
            raise ValueError('pose_topic and cmd_topic must be absolute ROS topic names')
        if pose_topic == cmd_topic:
            raise ValueError('pose_topic and cmd_topic cannot be identical')
        # Selection does NOT renumber positions: follower 2 gets target index 1.
        self.robot = Robot(self.follower_id, pose_topic, cmd_topic,
                           targets[self.follower_id - 1])
        self.robot.staging_target = staging_target(self.robot.target, spacing)
        self.frame = get('mocap_frame', '')
        self.started = time.monotonic()
        rospy.on_shutdown(self.stop)
        rospy.loginfo('Follower %d (%s): %s -> %s, goal=%s', self.follower_id,
                      self.mode, pose_topic, cmd_topic, self.robot.target)
        rospy.loginfo('Follower %d staging pose=%s', self.follower_id, self.robot.staging_target)
        rospy.loginfo('Mocap-to-experiment transform: dx=-%.3f m, dy=-%.3f m, rotation=%.2f deg',
                      self.tf_horizontal, self.tf_vertical, self.tf_rotation_deg)
        if self.dry_run:
            rospy.logwarn('DRY RUN: zero velocity commands only')

    def stop(self):
        # Several zero commands on shutdown or stale mocap, for reliability.
        for _ in range(5):
            self.robot.command()
            time.sleep(0.05)

    def checked_pose(self):
        pose, received, frame = self.robot.snapshot()
        if pose is None or received is None or time.monotonic() - received > self.mocap_timeout:
            return None
        if self.frame and frame != self.frame:
            raise RuntimeError('Unexpected mocap frame: %r' % frame)
        return mocap_to_experiment_pose(pose, self.tf_horizontal,
                                        self.tf_vertical, self.tf_rotation_deg)

    def step_robot(self, robot, pose):
        x, y, theta = pose
        fx, fy, target_yaw = robot.target
        sx, sy, _ = robot.staging_target
        yaw_error = wrap_angle(target_yaw-theta)

        if robot.state == 'COMPLETE':
            robot.command()
            return

        # Stage 1: navigate to the staging position (one extra spacing
        # behind the final formation slot), using the original controller.
        if robot.state in ('WAIT_FOR_MOCAP', 'MOVE_TO_POSITION'):
            robot.state = 'MOVE_TO_POSITION'
            dx, dy = sx-x, sy-y
            distance = math.hypot(dx, dy)
            if distance < self.distance_tol:
                robot.state = 'ALIGN_ORIENTATION'
                robot.command()
                return
            bearing_error = wrap_angle(math.atan2(dy, dx)-theta)
            w = max(-self.max_w, min(self.max_w, self.kp_w*bearing_error))
            v = (0.0 if abs(bearing_error) >= self.turn_threshold else
                 min(self.max_v, self.kp_v*distance)*max(0.0, math.cos(bearing_error)))
            robot.command(v, w)
            return

        # Stage 2: align at the staging position, before approaching
        # the occupied formation line.
        if robot.state == 'ALIGN_ORIENTATION':
            if math.hypot(sx-x, sy-y) >= self.distance_tol:
                robot.state = 'MOVE_TO_POSITION'
                robot.command()
            elif abs(yaw_error) < self.yaw_tol:
                robot.state = 'FINAL_APPROACH'
                robot.command()
            else:
                robot.command(0.0, max(-self.max_w, min(self.max_w, self.kp_w*yaw_error)))
            return

        # Stage 3: drive forward, parallel to the intended final heading.
        # Correct yaw, without turning around toward a noisy near-goal bearing.
        if robot.state == 'FINAL_APPROACH':
            distance = math.hypot(fx-x, fy-y)
            if distance < self.distance_tol:
                robot.state = 'VERIFY_POSE'
                robot.settling_since = time.monotonic()
                robot.command()
                return
            along = (fx-x)*math.cos(target_yaw) + (fy-y)*math.sin(target_yaw)
            if along <= 0:
                raise RuntimeError('Passed final target without reaching position tolerance')
            if abs(yaw_error) >= self.turn_threshold:
                robot.command(0.0, max(-self.max_w, min(self.max_w, self.kp_w*yaw_error)))
            else:
                v = min(self.max_v, self.kp_v*along)*max(0.0, math.cos(yaw_error))
                w = max(-self.max_w, min(self.max_w, self.kp_w*yaw_error))
                robot.command(v, w)
            return

        # Final verification; small position drift requires a new staged
        # approach, rather than turning toward another robot at close range.
        if robot.state == 'VERIFY_POSE':
            robot.command()
            distance = math.hypot(fx-x, fy-y)
            if distance >= self.distance_tol:
                robot.state = 'MOVE_TO_POSITION'
            elif abs(yaw_error) >= self.yaw_tol:
                robot.state = 'ALIGN_ORIENTATION'
            elif time.monotonic()-robot.settling_since >= self.settle_time:
                robot.state = 'COMPLETE'
                rospy.loginfo('Follower %d COMPLETE: position error=%.4f m, yaw error=%.2f deg',
                              robot.index, distance, math.degrees(yaw_error))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_log = 0.0
        while not rospy.is_shutdown():
            if time.monotonic()-self.started > self.timeout:
                raise RuntimeError('Follower %d calibration timed out' % self.follower_id)
            pose = self.checked_pose()
            if pose is None:
                self.robot.command()
                if time.monotonic()-last_log > 2:
                    rospy.logwarn('Follower %d waiting for fresh mocap on %s',
                                  self.follower_id, self.robot.pose_topic)
                    last_log = time.monotonic()
                rate.sleep()
                continue
            if self.dry_run:
                self.robot.command()
            else:
                self.step_robot(self.robot, pose)
            if self.robot.state == 'COMPLETE':
                self.stop()
                rospy.loginfo('FOLLOWER %d CALIBRATION SUCCESS', self.follower_id)
                return True
            if time.monotonic()-last_log > 2:
                goal = (self.robot.staging_target if self.robot.state in
                        ('WAIT_FOR_MOCAP', 'MOVE_TO_POSITION', 'ALIGN_ORIENTATION')
                        else self.robot.target)
                err = math.hypot(goal[0]-pose[0], goal[1]-pose[1])
                rospy.loginfo('F%d %s: distance error %.4f m',
                              self.follower_id, self.robot.state, err)
                last_log = time.monotonic()
            rate.sleep()
        return False


def main():
    rospy.init_node('formation_calibration')
    node = None
    try:
        node = FormationCalibration()
        node.run()
    except (ValueError, RuntimeError, KeyError, rospy.ROSException) as exc:
        rospy.logerr('FORMATION CALIBRATION ABORTED: %s', exc)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.stop()


if __name__ == '__main__':
    main()
