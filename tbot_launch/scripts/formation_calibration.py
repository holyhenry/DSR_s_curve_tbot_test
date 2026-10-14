#!/usr/bin/env python3
"""Mocap-based, one-shot TurtleBot3 formation positioning (ROS Noetic)."""
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


class Robot:
    def __init__(self, index, pose_topic, cmd_topic, target):
        self.index = index
        self.pose_topic = pose_topic
        self.cmd_topic = cmd_topic
        self.target = target
        self.pose = None
        self.last_received = None
        self.frame_id = None
        self.state = 'WAIT_FOR_MOCAP'
        self.settling_since = None
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback, queue_size=1)
        self.lock = threading.Lock()

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
    def __init__(self):
        get = lambda name, default: rospy.get_param('~' + name, default)
        self.mode = get('formation_type', 'platoon')
        self.dry_run = bool(get('dry_run', True))
        self.motion_mode = get('motion_mode', 'sequential')
        if self.motion_mode not in ('sequential', 'simultaneous'):
            raise ValueError('motion_mode must be sequential or simultaneous')
        self.distance_tol = float(get('position_tolerance', 0.005))
        self.yaw_tol = math.radians(float(get('yaw_tolerance_deg', 3.0)))
        self.max_v = float(get('max_linear_speed', 0.08))
        self.max_w = float(get('max_angular_speed', 0.5))
        self.kp_v = float(get('kp_linear', 0.8))
        self.kp_w = float(get('kp_angular', 1.5))
        self.rate_hz = float(get('control_rate', 20.0))
        self.timeout = float(get('calibration_timeout', 120.0))
        self.mocap_timeout = float(get('mocap_timeout', 0.5))
        self.settle_time = float(get('settle_time', 0.5))
        self.min_separation = float(get('min_separation', 0.20))
        self.turn_threshold = math.radians(float(get('turn_in_place_deg', 30.0)))
        self.leader = (float(get('leader_x', -0.5)), float(get('leader_y', 0.8)),
                       float(get('leader_yaw', -math.pi)))
        spacing = float(get('spacing', 0.25))
        arrow_a = float(get('arrow_a', 0.25))
        arrow_b = float(get('arrow_b', 0.15))
        numbers = [self.distance_tol, self.yaw_tol, self.max_v, self.max_w,
                   self.kp_v, self.kp_w, self.rate_hz, self.timeout,
                   self.mocap_timeout, self.settle_time, self.min_separation,
                   spacing, arrow_a, arrow_b]
        if not all(math.isfinite(n) and n > 0 for n in numbers):
            raise ValueError('Positive finite gains, limits, spacings and thresholds required')
        if self.max_v > 0.08:
            raise ValueError('max_linear_speed must not exceed 0.08 m/s')
        targets = formation_targets(self.mode, self.leader, spacing, arrow_a, arrow_b)
        config = get('robots', [])
        if len(config) < len(targets):
            raise ValueError('robots parameter must provide every active follower')
        active = config[:len(targets)]
        topics = [r['pose_topic'] for r in active]
        commands = [r['cmd_topic'] for r in active]
        if len(set(topics)) != len(topics) or len(set(commands)) != len(commands):
            raise ValueError('Each follower must have distinct mocap and cmd_vel topics')
        if any(not t.startswith('/') for t in topics + commands):
            raise ValueError('Use absolute ROS topic names')
        self.robots = [Robot(i+1, r['pose_topic'], r['cmd_topic'], target)
                       for i, (r, target) in enumerate(zip(active, targets))]
        self.frame = get('mocap_frame', '')
        self.started = time.monotonic()
        rospy.on_shutdown(self.stop_all)
        for robot in self.robots:
            rospy.loginfo('Follower %d: %s -> %s, goal=%s', robot.index,
                          robot.pose_topic, robot.cmd_topic, robot.target)
        if self.dry_run:
            rospy.logwarn('DRY RUN: no nonzero velocity will be sent')

    def stop_all(self):
        for _ in range(5):
            for robot in self.robots:
                robot.command()
            time.sleep(0.05)

    def checked_poses(self):
        poses = []
        now = time.monotonic()
        frames = set()
        for robot in self.robots:
            pose, received, frame = robot.snapshot()
            if pose is None or received is None or now-received > self.mocap_timeout:
                return None
            if self.frame and frame != self.frame:
                raise RuntimeError('Unexpected mocap frame for follower %d: %r' % (robot.index, frame))
            if frame:
                frames.add(frame)
            poses.append(pose)
        if len(frames) > 1:
            raise RuntimeError('Follower poses are reported in different coordinate frames')
        return poses

    def check_separation(self, poses):
        # Optional leader position assumed stationary at the specified reference pose.
        positions = [(p[0], p[1]) for p in poses]
        if rospy.get_param('~check_leader_clearance', True):
            positions.append(self.leader[:2])
        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                if math.hypot(positions[i][0]-positions[j][0],
                              positions[i][1]-positions[j][1]) < self.min_separation:
                    raise RuntimeError('Robot separation under minimum clearance: %d and %d' % (i, j))

    def step_robot(self, robot, pose):
        x, y, theta = pose
        tx, ty, target_yaw = robot.target
        dx, dy = tx-x, ty-y
        distance = math.hypot(dx, dy)
        final_yaw_error = wrap_angle(target_yaw-theta)
        if robot.state == 'COMPLETE':
            robot.command()
            return
        if robot.state in ('WAIT_FOR_MOCAP', 'MOVE_TO_POSITION'):
            robot.state = 'MOVE_TO_POSITION'
            if distance < self.distance_tol:
                robot.state = 'ALIGN_ORIENTATION'
                robot.command()
                return
            bearing_error = wrap_angle(math.atan2(dy, dx)-theta)
            w = max(-self.max_w, min(self.max_w, self.kp_w*bearing_error))
            if abs(bearing_error) >= self.turn_threshold:
                v = 0.0
            else:
                v = min(self.max_v, self.kp_v*distance)*max(0.0, math.cos(bearing_error))
            robot.command(v, w)
            return
        if robot.state == 'ALIGN_ORIENTATION':
            if distance >= self.distance_tol:
                robot.state = 'MOVE_TO_POSITION'
                robot.command()
            elif abs(final_yaw_error) < self.yaw_tol:
                robot.state = 'VERIFY_POSE'
                robot.settling_since = time.monotonic()
                robot.command()
            else:
                robot.command(0.0, max(-self.max_w, min(self.max_w, self.kp_w*final_yaw_error)))
            return
        if robot.state == 'VERIFY_POSE':
            robot.command()
            if distance >= self.distance_tol:
                robot.state = 'MOVE_TO_POSITION'
            elif abs(final_yaw_error) >= self.yaw_tol:
                robot.state = 'ALIGN_ORIENTATION'
            elif time.monotonic()-robot.settling_since >= self.settle_time:
                robot.state = 'COMPLETE'
                rospy.loginfo('Follower %d COMPLETE: position error=%.4f m, yaw error=%.2f deg',
                              robot.index, distance, math.degrees(final_yaw_error))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_log = 0.0
        while not rospy.is_shutdown():
            if time.monotonic()-self.started > self.timeout:
                raise RuntimeError('Calibration timed out')
            poses = self.checked_poses()
            if poses is None:
                self.stop_all()
                if time.monotonic()-last_log > 2:
                    rospy.logwarn('Waiting for fresh mocap from ALL active followers')
                    last_log = time.monotonic()
                rate.sleep()
                continue
            self.check_separation(poses)
            for robot, pose in zip(self.robots, poses):
                if self.motion_mode == 'sequential' and any(
                        prev.state != 'COMPLETE' for prev in self.robots[:robot.index-1]):
                    robot.command()
                    continue
                if self.dry_run:
                    robot.command()
                else:
                    self.step_robot(robot, pose)
            if all(r.state == 'COMPLETE' for r in self.robots):
                self.stop_all()
                rospy.loginfo('FORMATION CALIBRATION SUCCESS')
                return True
            if time.monotonic()-last_log > 2:
                for robot, pose in zip(self.robots, poses):
                    err = math.hypot(robot.target[0]-pose[0],robot.target[1]-pose[1])
                    rospy.loginfo('F%d %s: distance error %.4f m', robot.index, robot.state, err)
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
            node.stop_all()


if __name__ == '__main__':
    main()
