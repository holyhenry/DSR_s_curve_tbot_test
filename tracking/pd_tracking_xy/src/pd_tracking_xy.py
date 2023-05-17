import numpy as np
import time

# ros library
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

class PID:

    def __init__(self, dt, kp_linear = 10.0, ki_linear = 1.0, kd_linear = 1.0,
                       kp_angular = 5.0, ki_angular = 0.0, kd_angular = 0.):

        self.dt = dt
        self.kp_linear = kp_linear
        self.ki_linear = ki_linear
        self.kd_linear = kd_linear

        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular

        self.pos_error_last = 0
        self.ang_error_last = 0

        # linear & angular control input in P, I, D terms
        self.linear_u = np.zeros(3)
        self.angular_u = np.zeros(3)

    def pid(self, robot, s_desired):

        # linear control input calculation
        p_actual = robot.s[:2]
        p_desired = s_desired[:2]
        pos_error = np.linalg.norm(p_desired - p_actual)
                
        self.linear_u[0] = self.kp_linear * pos_error
        self.linear_u[1] += self.ki_linear * (pos_error+self.pos_error_last)/2 * self.dt
        self.linear_u[2] = self.kd_linear * (pos_error-self.pos_error_last)

        self.pos_error_last = pos_error

        linear_control_input = np.sum(self.linear_u) # v, m/s

        # angular control input calculation
        a_actual = robot.s[2]
        a_desired = np.arctan2(p_desired[1]-p_actual[1], p_desired[0]-p_actual[0])
        ang_error = a_desired - a_actual

        self.angular_u[0] = self.kp_angular * ang_error
        self.angular_u[1] += self.ki_angular * (ang_error+self.ang_error_last)/2 * self.dt
        self.angular_u[2] = self.kd_angular * (ang_error-self.ang_error_last)

        self.ang_error_last = ang_error

        angular_control_input = np.sum(self.angular_u) # w, rad/s

        return np.array([linear_control_input, angular_control_input])

class PD:

    def __init__(self, dt, kp = 4.0, kd = 1.0, alpha=5.0):

        self.dt = dt
        self.alpha = alpha
        
        self.kp = kp
        self.kd = kd

        self.ex_last = 0.0
        self.ey_last = 0.0

        self.ux_ddot = 0.0
        self.uy_ddot = 0.0

        self.v = 0.0
        self.w = 0.0

    def pd(self, robot, s_desired):
        
        p_actual  = robot.get_state()[:2]
        p_desired = s_desired[:2]
        theta     = robot.get_state()[2]
        
        ex = (p_desired - p_actual)[0]
        ey = (p_desired - p_actual)[1]
        
        self.ux_ddot = self.alpha*((ex-self.ex_last)/self.dt + self.kp*ex) - self.kp*robot.get_velocity()*np.cos(theta)
        self.uy_ddot = self.alpha*((ey-self.ey_last)/self.dt + self.kp*ey) - self.kp*robot.get_velocity()*np.sin(theta)

        # dynamic fb linearization map
        aw_2_xy = np.array([[np.cos(robot.get_state()[2]), -robot.get_velocity()*np.sin(robot.get_state()[2])],
                            [np.sin(robot.get_state()[2]), robot.get_velocity()*np.cos(robot.get_state()[2])]])
        
        self.v += (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[0]*self.dt
        self.w = (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[1]

        self.ex_last = ex
        self.ey_last = ey

        return np.array([self.v, self.w])

class tracking_node:

    def __init__(self) -> None:

        self.state = np.zeros(3) # self x,y,yaw
        self.leader_state = np.zeros(2) # leader x,y
        self.leader_states = []

    def aprilTagCallback(self, data):

        '''
        global (+)x = aprilTag (+)x - 0.03m
        global (+)y = aprilTag (+)z
        '''
        cam_pose_offset = 0.03
        leader_x = data.x-cam_pose_offset
        leader_y = data.z
        self.leader_state = np.array([leader_x, leader_y])
        self.leader_states.append(self.leader_state)

    def odometeryCallback(self, data):

        '''
        global (+)x = robotOdom (-)y
        global (+)y = robotOdom (+)x  
        global (+)yaw = robotOdom (+)yaw + np.pi/2
        '''
        self_x = -data.pose.pose.position.y
        self_y = data.pose.pose.position.x
        self_yaw = data.pose.pose.orientation.z + np.pi/2
        self.state = np.array([self_x, self_y, self_yaw])

    def pubCmdVel(self, pub, ctrl_linear_vel=0.0, ctrl_angular_vel=0.0):

        twist = Twist()
        twist.linear.x = ctrl_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = ctrl_angular_vel
        pub.publish(twist)

    def getTrackingTarget(self, distance):

        target = np.zeros(2)
        find_target = False
        leader_traj = np.array(self.leader_states)

        for i in range(len(leader_traj)-1, -1, -1):
            travel_length = leader_traj[i][:]-self.leader_state[:]

            if np.linalg.norm(travel_length, ord=2)>=distance:
                target = leader_traj[i][:]
                find_target = True
                break

        if not find_target:
            dx = self.state[0]-self.leader_state[0]
            dy = self.state[1]-self.leader_state[1]
            theta = np.arctan2(dy, dx)
            target[0] = self.leader_state[0] + distance*np.cos(theta)
            target[1] = self.leader_state[1] + distance*np.sin(theta)

        return target

    def run(self):

        rospy.init_node('pd_tracking_xy')
        rate = rospy.Rate(25)

        rospy.Subscriber("/odom", Odometry, self.odometeryCallback, queue_size=1)
        rospy.Subscriber("/april_data", Point, self.aprilTagCallback, queue_size=1)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        while not rospy.is_shutdown():
            
            
            
            self.pubCmdVel(pub) # need modify!!!!!!
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    
    Tracking_node = tracking_node()
    Tracking_node.run()