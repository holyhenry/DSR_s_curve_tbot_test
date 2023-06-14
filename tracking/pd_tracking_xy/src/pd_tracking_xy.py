import numpy as np
import bezier

# ros library
import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry

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
        self.w_last = 0.0

        self.v = 0.0
        self.w = 0.0
    
    def lowPass(self,u,y_last):
        lowPassGain = 0.95
        y = lowPassGain*y_last + (1 - lowPassGain)*u
        return y

    def invMapGain(self,v,thre,a):
        k = (1/thre)**a/thre
        if np.abs(v) >= thre:
            y = 1/v
        else:
            y = 1/thre
        """ 
        if v < 0:
            y = -((-k*v)**(1/a))
        else:
            y = (k*v)**(1/a)
        """
        return y

    def omegaLim(self, v):
        w_max = min(4*np.abs(v),1.58)
        return w_max
    
    def pd(self, states, velocity, goal, dataPub, leader_states):
        '''
        states are represented in follower local frame
        '''
        # p_actual  = state[:2]
        p_desired = goal[:2]
        # theta     = state[2]

        ex = p_desired[0]
        ey = p_desired[1]
        ex = self.lowPass(ex,self.ex_last)
        ey = self.lowPass(ey,self.ey_last)

        ex_dot = (ex - self.ex_last)/self.dt
        ey_dot = (ey - self.ey_last)/self.dt

        data = Twist()
        data.linear.x = ey
        data.linear.y = ey_dot
        
        x_dot = self.v
        y_dot = 0
        #self.ux_ddot = self.alpha*(ex_dot + self.kp*ex) - self.kp*x_dot
        x_bar = self.alpha*(ex_dot + self.kp*ex) - self.kp*x_dot
        
        # self.uy_ddot = self.alpha*(ey_dot + self.kp*ey) - self.kp*y_dot
        y_bar = self.alpha*(ey_dot + self.kp*ey) - self.kp*y_dot
        #print('x bar cmd:',x_bar)
        #print('y bar cmd',y_bar)

        # dynamic fb linearization map
        # aw_2_xy = np.array([[np.cos(theta), -tmp_vel*np.sin(theta)],
                           # [np.sin(theta), tmp_vel*np.cos(theta)]])
                           
        #self.v += (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[0]*self.dt
        #self.w = (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[1]

        self.v += (x_bar)*self.dt
        self.v = np.clip(self.v,-0.2,0.2)

        self.w = self.invMapGain(velocity,0.05,1)*y_bar
        w_max = self.omegaLim(velocity)
        data.angular.x = w_max
        data.angular.y = velocity

        self.w = np.clip(self.w,-w_max,w_max)
        
        data.linear.z = self.w
        dataPub.publish(data)

        self.ex_last = ex
        self.ey_last = ey
        self.w_last = self.w
        return np.array([self.v, self.w])

class tracking_node:

    def __init__(self) -> None:

        self.state = np.zeros(3)        # self x,y,yaw
        self.state_last = np.zeros(3)   
        self.velocity = 0.0             
        self.leader_state = np.zeros(2) # leader x,y
        self.states = []
        self.leader_states = []
        self.target_status = -1

    def getLeaderStates(self):

        return np.array(self.leader_states).copy()

    def getStates(self):

        return np.array(self.states).copy()

    def initNode(self, freq):

        rospy.init_node('pd_tracking_xy')
        rate = rospy.Rate(int(freq))
        ns = rospy.get_namespace()

        rospy.Subscriber(ns + "odom", Odometry, self.odometeryCallback, queue_size=1)
        rospy.Subscriber(ns + "april_data", Point, self.aprilTagCallback, queue_size=1)
        rospy.Subscriber(ns + "april_data_multi",Float32MultiArray, self.multiAprilTagCallback, queue_size=1)
        pub = rospy.Publisher(ns + "cmd_vel", Twist, queue_size=1)
        pub_data = rospy.Publisher(ns + "data", Twist, queue_size=1)
        pub_target = rospy.Publisher(ns + "target", Point, queue_size=1)

        return pub, pub_data, pub_target, rate

    def aprilTagCallback(self, data):
        '''
        not using global frame right now!
        global (+)x = aprilTag (+)x - 0.03m
        global (+)y = aprilTag (+)z
        robotOdom (+)x = aprilTag (+)z
        robotOdom (+)y = aprilTag (-)(x-0.03m)
        '''
        cam_pose_offset = 0.03
        leader_x = data.z
        leader_y = -(data.x-cam_pose_offset)
        self.leader_state = np.array([leader_x, leader_y])
        self.leader_states.append(self.leader_state)

    def multiAprilTagCallback(self, data):
        
        leader_x  = 0.0
        leader_y  = 0.0
        num_tag   = 3
        tag_space = 5
        cam_pose_offset = 0.03
        multi_tag = data.data
        for i in range(len(multi_tag)):
            if (i%tag_space==0 and (i//tag_space)%2==1):
                x   = multi_tag[i+3]
                y   = multi_tag[i+1]
                phi = multi_tag[i+4]
                id  = multi_tag[i]
                print('id:',multi_tag[i%tag_space])
                print('infered pos:',self.transformTag2Middle(x,y,phi,id))

    def transformTag2Middle(self, x, y, phi, id, num_tag=3, d=0.038):
        '''
        <I/O>
        x,y: detected side tag pos
        phi: side tag orientation
        id: april tag id
        d: hardware goemytry (m)
        x1_hat,y1_hat: infered middle tag pos
        <Other>
        PHI: angle between l, l_hat
        theta: angle between l, x
        theta_hat: angle between l_hat, x1_hat
        '''
        l     = np.sqrt(x**2+y**2)
        l_hat = np.sqrt(d**2+l**2-2*d*l*np.cos(phi+0.2616))
        PHI   = np.arccos((l**2)/(2*l*l_hat))
        theta = np.arctan2(np.abs(y),x)
        if (id%num_tag == 0):
            theta_hat = theta-PHI
        if (id%num_tag == 2):
            theta_hat = PHI-theta
        x1_hat = l_hat*np.cos(theta_hat)
        y1_hat = l_hat*np.sin(theta_hat)

        return x1_hat, y1_hat

    def odometeryCallback(self, data):
        '''
        not using global frame right now!
        global (+)x = robotOdom (-)y
        global (+)y = robotOdom (+)x  
        global (+)yaw = robotOdom (+)yaw + np.pi/2
        '''
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        (_, _, self_yaw) = transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])
        self_x = data.pose.pose.position.x
        self_y = data.pose.pose.position.y
        # self_x = 0.0 # local frame
        # self_y = 0.0 # local frame
        self.state = np.array([self_x, self_y, self_yaw])
        self.states.append(self.state)

        self.velocity = data.twist.twist.linear.x
        
    def pubCmdVel(self, pub, ctrl_linear_vel=0.0, ctrl_angular_vel=0.0):
        
        ctrl_linear_vel  = np.clip(ctrl_linear_vel, -0.20, 0.20)
        ctrl_angular_vel = np.clip(ctrl_angular_vel, -1.58, 1.58)
        twist = Twist()
        twist.linear.x = ctrl_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = ctrl_angular_vel
        pub.publish(twist)

    def updateLocalFrame(self):
        '''
        get new readings for leader trajectory
        '''
        if (len(self.states)>=2):
            del_theta = self.state[:2] -  np.array(self.states)[-2,2] # - np.array(self.states)[-2,2]
            del_pos = self.state[:2] - np.array(self.states)[-2,:2] # - np.array(self.states)[-2,:2]
            print('del_pos~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~',del_pos)
            # apply frame translation
            self.leader_states[:-1] = np.add(self.leader_states[:-1], -del_pos)

            # frame rotation matrix T 
            T = np.array([[np.cos(del_theta), np.sin(del_theta)],
                        [-np.sin(del_theta), np.cos(del_theta)]])
            
            # apply rotaiton matrix T
            # self.leader_states[:-1] = np.einsum('ij,kj->ki', T, self.leader_states[:-1])

    def getUnitCircleTarget(self, pub, distance):

        target = np.zeros(2)
        find_target = False
        leader_traj = np.array(self.leader_states)

        for i in range(len(leader_traj)-1, -1, -1):
            travel_length = leader_traj[i][:]-self.leader_state[:]

            if np.linalg.norm(travel_length, ord=2)>=distance:
                #print("YES!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                target = leader_traj[i][:]
                self.target_status = 1
                find_target = True
                break

        if not find_target:
            #dx = self.state[0]-self.leader_state[0]
            #dy = self.state[1]-self.leader_state[1]
            dx = self.leader_state[0]
            dy = self.leader_state[1]
            theta = np.arctan2(dy, dx)
            #target[0] = self.leader_state[0] + distance*np.cos(theta)
            #target[1] = self.leader_state[1] + distance*np.sin(theta)
            target[0] = self.leader_state[0] - distance*np.cos(theta)
            target[1] = self.leader_state[1] - distance*np.sin(theta)
            self.target_status = 0

        # record topic 
        target_point = Point()
        target_point.x = target[0]
        target_point.y = target[1]
        pub.publish(target_point)

        return target

    def getBezierTarget(self, distance):
        '''
        return: s_l(t)-s_f(t) and desired heading
        '''
        indx = 1
        target = np.zeros(2)
        threshold = 5*1e-2
        check_length = 300 # Use larger number if interbot_distance is longer

        while(indx<check_length):
            dist = np.linalg.norm(self.state[:2]-self.leader_states[-indx,:2], ord=2)
            
            if (dist<=threshold or indx==len(self.leader_states)):
                bezier_states = np.asfortranarray([np.append(self.state[0], self.leader_states[-indx:,0]),
                                                   np.append(self.state[1], self.leader_states[-indx:,1])])
                curve = bezier.Curve(bezier_states, degree=indx)

                # evaluate a desired heading angle
                x = (curve.evaluate(0.05).reshape(2)-self.state[:2])[0]
                y = (curve.evaluate(0.05).reshape(2)-self.state[:2])[1]
                theta_s = np.arctan2(y, x)

                e_s = curve.length - distance
                target[0] = e_s*np.cos(theta_s)
                target[1] = e_s*np.sin(theta_s)
                
                self.ctrl_status = 2 if dist<=threshold else 3
                return target, True
            
            indx += 1

        # no feasible fitting point within the 'check_length'
        return None, False

    def run(self):
        
        freq = 5.0
        cmdVelPub, dataPub, targetPub, rate = self.initNode(freq)

        # controller setups
        dt = 1.0/freq
        ctrl  = PD(dt=dt, kp = 0.3, kd = 1.0, alpha=0.5)

        while not rospy.is_shutdown():

            #self.updateLocalFrame()

            #if (len(self.leader_states)>0):
            #    print('----------------------------------transformed leader start state :) index -1',self.leader_states[-1])
            #    print('----------------------------------transformed leader start state :) index 0',self.leader_states[0])
            #print("self.target_status", self.target_status)

            goal = self.getUnitCircleTarget(targetPub, distance=0.35)

            u = ctrl.pd(self.getStates(), self.velocity, goal, dataPub, self.getLeaderStates())
            
            print('leader state',self.leader_state,'goal ', goal)
            #print('vel cmd', u[0],'rot cmd',u[1])
            print('----------------------------------------------')

            #self.pubCmdVel(cmdVelPub, u[0], u[1])
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':

    Tracking_node = tracking_node()
    Tracking_node.run()


