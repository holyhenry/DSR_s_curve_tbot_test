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

        # current state (local)
        self.state        = np.zeros(3) # x,y,yaw
        self.state_last   = np.zeros(3)   
        self.velocity     = 0.0             
        self.leader_state = np.zeros(2) # leader x,y,yaw

        # odom (global)
        self.states = []
        # tag info (global)
        self.leader_states = []
        # tag info (local)
        self.leader_states_local = []

        self.target_status = -1
        self.cum_angle = 0

    def getLeaderStates(self):

        return np.array(self.leader_states).copy()

    def getStates(self):

        return np.array(self.states).copy()

    def initNode(self, freq):

        rospy.init_node('pd_tracking_xy')
        rate = rospy.Rate(int(freq))
        ns   = rospy.get_namespace()

        rospy.Subscriber(ns + "odom", Odometry, self.odometeryCallback, queue_size=1)
        rospy.Subscriber(ns + "april_data", Point, self.aprilTagCallback, queue_size=1)
        rospy.Subscriber(ns + "april_data_multi",Float32MultiArray, self.multiAprilTagCallback, queue_size=1)
        pub        = rospy.Publisher(ns + "cmd_vel", Twist, queue_size=1)
        pub_data   = rospy.Publisher(ns + "data", Twist, queue_size=1)
        pub_target = rospy.Publisher(ns + "target", Point, queue_size=1)
        pub_l_traj = rospy.Publisher(ns + "l_traj", Float32MultiArray, queue_size=1)

        return pub, pub_data, pub_target, pub_l_traj ,rate
    
    def checkInputs(self):

        while not (len(self.states)>0 and len(self.leader_states)>0):

            rospy.logwarn("waiting for data")

    def aprilTagCallback(self, data):
        '''
        robotOdom (+)x = aprilTag (+)z
        robotOdom (+)y = aprilTag (-)(x-0.03m)
        '''
        cam_pose_offset = 0.03
        leader_x = data.z
        leader_y = -(data.x-cam_pose_offset)
        self.leader_state = np.array([leader_x, leader_y])
        self.leader_states.append(self.leader_state)

    def multiAprilTagCallback(self, data):
        
        multi_tag = data.data
        # initialize
        count      = 0
        num_tag    = 3
        tag_space  = 5
        tag_x   = 0.0
        tag_y   = 0.0
        tag_phi = 0.0
        follower_indx   = 0
        cam_pose_offset = 0.03

        for i in range(len(multi_tag)):
            if (i%tag_space == 0 and (multi_tag[i]//num_tag == follower_indx)):
                count += 1
                x   = multi_tag[i+3]
                y   = -(multi_tag[i+1]-cam_pose_offset)
                phi = multi_tag[i+4]
                id  = multi_tag[i]
                infered_x, infered_y, infered_phi = self.transformTag2Middle(x,y,phi,id)
                tag_x   += infered_x
                tag_y   += infered_y
                tag_phi += infered_phi
        
        if (count != 0):
            tag_x   /= count
            tag_y   /= count
            tag_phi /= count
            self.leader_state = self.homoTrans2BotCenter(np.array([tag_x,tag_y,tag_phi]))
            self.leader_states.append(self.homoTrans2Global(self.leader_state))

    def transformTag2Middle(self, x, y, alpha, id, num_tag=3, d=0.038):

        if (id%num_tag == 0):
            x1_hat = x + d*np.cos(-np.pi/2+alpha+0.2616)
            y1_hat = y + d*np.sin(-np.pi/2+alpha+0.2616)
            alpha  += 30/180*np.pi
            return x1_hat, y1_hat, alpha
        
        if (id%num_tag == 2):
            x1_hat = x + d*np.cos(np.pi/2+alpha-0.2616)
            y1_hat = y + d*np.sin(np.pi/2+alpha-0.2616)
            alpha  -= 30/180*np.pi
            return x1_hat, y1_hat, alpha
        
        else:
            return x, y, alpha
    
    def homoTrans2BotCenter(self, state):

        del_theta = state[2]
        del_pos   = state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                               [np.sin(del_theta), np.cos(del_theta)]]) 
        trans_vec  = np.atleast_2d(del_pos).T   
        T = np.vstack([np.hstack([rot_matrix, trans_vec]), np.array([0.,0.,1.])])   

        tags_2_bot_centor = np.array([0.14, 0.])
        homo_state = np.hstack([tags_2_bot_centor, 1.])
        new_state = (T@homo_state)[:2]

        return new_state       

    def homoTrans2Global(self, state):

        del_theta = self.state[2]
        del_pos   = self.state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                               [np.sin(del_theta), np.cos(del_theta)]]) 
        trans_vec  = np.atleast_2d(del_pos).T   
        T = np.vstack([np.hstack([rot_matrix, trans_vec]), np.array([0.,0.,1.])])   

        homo_state = np.hstack([state, 1.])
        new_state = (T@homo_state)[:2]

        return new_state
    
    def homoInvTrans2Local(self, states):

        del_theta = self.state[2]
        del_pos   = self.state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                            [np.sin(del_theta), np.cos(del_theta)]])
        trans_vec = np.atleast_2d(del_pos).T
        T = np.vstack([np.hstack([rot_matrix, trans_vec]), np.array([0.,0.,1.])])

        ones = np.atleast_2d(np.ones(np.array(states).shape[0])).T
        try:
            homo_states = np.hstack([states, ones])
        except:
            print('size mismatch!!')
            ones = np.atleast_2d(np.ones(np.array(states).shape[0])).T
            homo_states = np.hstack([states, ones])
        new_states = np.einsum('ij,kj->ki', np.linalg.inv(T), homo_states)[:,:2]
    
        return new_states

    def odometeryCallback(self, data):
        '''
        not using global frame right now!
        global (+)x = robotOdom (-)y
        global (+)y = robotOdom (+)x  
        global (+)yaw = robotOdom (+)yaw + np.pi/2
        '''
        self_x = data.pose.pose.position.x
        self_y = data.pose.pose.position.y
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        (_, _, self_yaw) = transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])

        self.velocity = data.twist.twist.linear.x
        self.state    = np.array([self_x, self_y, self_yaw])
        self.states.append(self.state)

    def pubLeaderTraj(self, pub):
        
        infered_leader_traj = Float32MultiArray()
        infered_leader_traj.data = self.getLeaderStates().flatten()
        pub.publish(infered_leader_traj)
        
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

    def getUnitCircleTarget(self, targetPub, distance):

        target = np.zeros(2)
        find_target = False
        leader_traj = np.array(self.leader_states_local)

        for i in range(len(leader_traj)-1, -1, -1):
            travel_length = leader_traj[i][:]-self.leader_state[:]

            if np.linalg.norm(travel_length, ord=2)>=distance:
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

        # record
        target_global  = self.homoTrans2Global(target)
        target_point   = Point()
        target_point.x = target_global[0]
        target_point.y = target_global[1]
        target_point.z = self.target_status
        targetPub.publish(target_point)

        return target

    def getBezierTarget(self, targetPub, distance):
        '''
        return: s_l(t)-s_f(t) and desired heading
        '''
        indx   = 1
        target = np.zeros(2)
        threshold    = 0.03
        check_length = 150 # Might need a larger number if interbot distance is longer
        leader_traj  = np.array(self.leader_states_local)

        while(indx<check_length and len(leader_traj)!=0):
            # dist = np.linalg.norm(self.state[:2]-leader_traj[-indx,:2], ord=2)
            dist = np.linalg.norm(leader_traj[-indx,:2], ord=2)
            
            if (dist<=threshold or indx==len(leader_traj)):
                # bezier_states = np.asfortranarray([np.append(self.state[0], leader_traj[-indx:,0]),
                #                                    np.append(self.state[1], leader_traj[-indx:,1])])
                bezier_states = np.asfortranarray([np.append(0., leader_traj[-indx:,0]),
                                                   np.append(0., leader_traj[-indx:,1])])
                curve = bezier.Curve(bezier_states, degree=indx)

                # evaluate a desired heading angle
                # x = (curve.evaluate(0.05).reshape(2)-self.state[:2])[0]
                # y = (curve.evaluate(0.05).reshape(2)-self.state[:2])[1]
                x = (curve.evaluate(0.05).reshape(2))[0]
                y = (curve.evaluate(0.05).reshape(2))[1]
                theta_s = np.arctan2(y, x)

                e_s = curve.length - distance
                target[0] = e_s*np.cos(theta_s)
                target[1] = e_s*np.sin(theta_s)
                
                self.target_status = 2 if dist<=threshold else 3

                # record
                target_global  = self.homoTrans2Global(target)
                target_point   = Point()
                target_point.x = target_global[0]
                target_point.y = target_global[1]
                target_point.z = self.target_status
                targetPub.publish(target_point)

                return target, True
            
            indx += 1

        # no feasible fitting point within the 'check_length'
        return None, False

    def run(self):
        
        freq = 10.0
        cmdVelPub, dataPub, targetPub, lTrajPub, rate = self.initNode(freq)

        # controller setups
        dt = 1.0/freq
        ctrl  = PD(dt=dt, kp = 0.3, kd = 1.0, alpha=0.6)
        self.checkInputs()

        while not rospy.is_shutdown():

            self.leader_states_local = self.homoInvTrans2Local(self.leader_states)

            goal, getTarget = self.getBezierTarget(targetPub, distance=0.40)
            if not getTarget:
               print('bezier fail')
               goal = self.getUnitCircleTarget(targetPub, distance=0.40)
            # goal = self.getUnitCircleTarget(targetPub, distance=0.40)
            
            u = ctrl.pd(self.getStates(), self.velocity, goal, dataPub, self.getLeaderStates())
            
            # print("self.target_status", self.target_status)
            # print('leader state',self.leader_state,'goal ', goal)
            # print('----------------------------------------------')

            self.pubLeaderTraj(lTrajPub)
            self.pubCmdVel(cmdVelPub, u[0], u[1])
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':

    Tracking_node = tracking_node()
    Tracking_node.run()


