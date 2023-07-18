import numpy as np

# ros library
import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class PD:

    def __init__(self, dt, kp=4.0, kd=1.0, alpha=1.0, dist=0.4):

        self.dt = dt
        self.alpha = alpha
        self.beta = 1.0 #TEST
        
        self.kp = kp
        self.kd = kd

        self.ex_last = 0.0
        self.ey_last = 0.0
        self.es_last = 0.0

        self.ux_ddot = 0.0
        self.uy_ddot = 0.0
        self.w_last = 0.0

        self.v = 0.0
        self.w = 0.0

        self.dist = dist # inter-robot distance
    
    def lowPass(self, y_current, y_last, lowPassGain = 0.2):
        
        y = lowPassGain*y_last + (1 - lowPassGain)*y_current

        return y

    def invMapGain(self, vel, thre, a):

        if np.abs(vel) >= thre:
            y = 1/vel
        else:
            y = 1/thre
        """ 
        k = (1/thre)**a/thre
        if vel < 0:
            y = -((-k*vel)**(1/a))
        else:
            y = (k*vel)**(1/a)
        """
        return y

    def omegaLim(self, v):

        w_max = min(4*np.abs(v),1.58)
        return w_max
    
    def pd(self, velocity, goal, dataPub):
        '''
        states are represented in follower local frame
        '''
        # p_actual  = state[:2]
        p_desired = goal[:2]
        # theta     = state[2]

        ex = p_desired[0]
        ey = p_desired[1]
        ex = self.lowPass(ex,self.ex_last,lowPassGain=0.5)
        ey = self.lowPass(ey,self.ey_last,lowPassGain=0.5)

        ex_dot = (ex - self.ex_last)/self.dt
        ey_dot = (ey - self.ey_last)/self.dt
        
        x_dot = self.v
        y_dot = 0
        # self.ux_ddot = self.alpha*(ex_dot + self.kp*ex) - self.kp*x_dot
        # self.uy_ddot = self.alpha*(ey_dot + self.kp*ey) - self.kp*y_dot
        ux_ddot = self.alpha*(ex_dot + self.kp*ex) - self.kp*x_dot
        uy_ddot = self.alpha*(ey_dot + self.kp*ey) - self.kp*y_dot

        # dynamic fb linearization map
        # aw_2_xy = np.array([[np.cos(theta), -tmp_vel*np.sin(theta)],
                           # [np.sin(theta), tmp_vel*np.cos(theta)]])
                           
        # self.v += (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[0]*self.dt
        # self.w = (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[1]
        self.v += (ux_ddot)*self.dt
        self.v = np.clip(self.v,-0.2,0.2)

        self.w = self.invMapGain(velocity,0.04,1)*uy_ddot
        w_max  = self.omegaLim(velocity)
        self.w = np.clip(self.w,-w_max,w_max)
        
        # record
        data = Twist()
        data.linear.x  = ey
        data.linear.y  = ex
        data.angular.x = ey_dot
        data.angular.y = ex_dot
        data.linear.z  = 0.0
        dataPub.publish(data)

        self.ex_last = ex
        self.ey_last = ey
        self.w_last  = self.w
        return np.array([self.v, self.w])
    
    def pd_s(self, velocity, curve_length_s, theta_s, dataPub):
        '''
        states are represented in follower local frame
        '''
        es = curve_length_s - self.dist
        es = self.lowPass(es, self.es_last, lowPassGain=0.5)
        us_dot = self.alpha*es*self.beta + 0.4*(self.v + self.beta*(es - self.es_last)/self.dt)
        # p_actual  = state[:2]
        # p_desired = goal[:2]
        # theta     = state[2]

        ex = us_dot*np.cos(theta_s)
        ey = us_dot*np.sin(theta_s)
        ex = self.lowPass(ex, self.ex_last, lowPassGain=0.3)
        ey = self.lowPass(ey, self.ey_last, lowPassGain=0.3)
        ex_dot = (ex - self.ex_last)/self.dt
        ey_dot = (ey - self.ey_last)/self.dt
        
        x_dot = self.v
        y_dot = 0
        # self.ux_ddot = self.alpha*(ex_dot + self.kp*ex) - self.kp*x_dot
        # self.uy_ddot = self.alpha*(ey_dot + self.kp*ey) - self.kp*y_dot
        x_bar = (ex_dot + self.kp*ex) - self.kp*x_dot
        y_bar = (ey_dot + self.kp*ey) - self.kp*y_dot

        # dynamic fb linearization map
        # aw_2_xy = np.array([[np.cos(theta), -tmp_vel*np.sin(theta)],
                           # [np.sin(theta), tmp_vel*np.cos(theta)]])
                           
        # self.v += (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[0]*self.dt
        # self.w = (np.linalg.pinv(aw_2_xy)@np.array([self.ux_ddot, self.uy_ddot]))[1]
        self.v += (x_bar)*self.dt
        self.v = np.clip(self.v,-0.2,0.2)

        self.w = self.invMapGain(velocity,0.04,1)*y_bar
        w_max  = self.omegaLim(velocity)
        self.w = np.clip(self.w,-w_max,w_max)
        
        # record
        data = Twist()
        data.linear.x  = es
        data.linear.y  = self.es_last
        data.linear.z  = 0.0
        data.angular.x = 0.0
        data.angular.y = 0.0
        dataPub.publish(data)

        self.es_last = es
        self.ex_last = ex
        self.ey_last = ey
        self.w_last  = self.w
        return np.array([self.v, self.w])
    
class DSR:

    def __init__(self, dt, kp=4.0, kd=1.0, alpha=1.0, beta=0.5, dist=0.4):

        self.dt    = dt
        self.alpha = alpha
        self.beta  = beta

        self.kp = kp
        self.kd = kd

        self.x_dot_last = 0.0
        self.y_dot_last = 0.0
        
        self.e_s_last = 0.0
        self.e_x_last = 0.0
        self.e_y_last = 0.0

        self.v = 0.0
        self.w = 0.0
        
        self.dist = dist # inter-robot distance
        self.leader_state_last = np.zeros(2)

    def lowPass(self, y_current, y_last, lowPassGain=0.2):

        return lowPassGain*y_last + (1 - lowPassGain)*y_current
    
    def invMapGain(self, vel, thre, a):

        if np.abs(vel) >= thre:
            y = 1/vel
        else:
            y = 1/thre
        """ 
        k = (1/thre)**a/thre
        if vel < 0:
            y = -((-k*vel)**(1/a))
        else:
            y = (k*vel)**(1/a)
        """
        return y

    def omegaLim(self, v):

        w_max = min(4*np.abs(v),1.58)
        return w_max
    
    def dsr(self, curve_length_s, theta_s, velocity, theta, leader_state):

        # equation (2)
        e_s = curve_length_s - self.dist
        e_s = self.lowPass(e_s, self.e_s_last, lowPassGain=0.0)
        # sf_dot = self.beta*velocity + (1-self.beta)*leader_velocity + self.alpha*self.beta*e_s 
        sf_dot = self.alpha*self.beta*e_s #+ velocity + self.beta*(e_s-self.e_s_last)/self.dt 

        # equation (3)
        x_dot = sf_dot*np.cos(theta_s)
        y_dot = sf_dot*np.sin(theta_s)
        # x_ddot = ((x_dot-self.x_dot_past)/self.dt + self.kp*x_dot) - self.kp*velocity*np.cos(theta)
        # y_ddot = ((y_dot-self.y_dot_past)/self.dt + self.kp*y_dot) - self.kp*velocity*np.sin(theta)
        x_bar = ((x_dot-self.x_dot_last)/self.dt + self.kp*x_dot) - self.kp*velocity
        y_bar = ((y_dot-self.y_dot_last)/self.dt + self.kp*y_dot)

        # # equation (4)
        # aw_2_xy = np.array([[np.cos(theta), -velocity*np.sin(theta)],
        #                     [np.sin(theta), velocity*np.cos(theta)]])
        
        # # equation (5)
        # self.v += (np.linalg.pinv(aw_2_xy)@np.array([self.x_ddot, self.y_ddot]))[0]*self.dt
        # self.w = (np.linalg.pinv(aw_2_xy)@np.array([self.x_ddot, self.y_ddot]))[1]
        self.v += (x_bar)*self.dt
        self.v = np.clip(self.v,-0.2,0.2)

        self.w = self.invMapGain(velocity,0.04,1)*y_bar
        w_max  = self.omegaLim(velocity)
        self.w = np.clip(self.w,-w_max,w_max)
        
        self.e_s_last   = e_s
        self.x_dot_last = x_dot
        self.y_dot_last = y_dot
        self.leader_state_last = leader_state
        return np.array([self.v, self.w])

class Bezier:
    
    def __init__(self, x, y):
        
        self.x = x
        self.y = y

        self.CELLS = 100 # Total number of divisions for Bezier curve
        self.t = np.linspace(0,1,self.CELLS) # Parametric variables

        self.nCPTS = np.size(self.x,0) # Total number of control points
        self.n = self.nCPTS - 1        # Total number of segments
        self.i = 0                     # Control point counter
        self.b = []                    # Collect Bernstein Basis Polynomial

        self.BezierCurve = np.zeros((self.CELLS,2))
        self.curveLength = 0.0
        
    def Ni(self): 
        '''
        Binomial Coefficients
        '''
        n = self.n
        i = self.i
        return np.math.factorial(n) / (np.math.factorial(i) * np.math.factorial(n-i))
    
    def basisFunction(self):
        '''
        Bernstein Basis Polynomial
        '''
        t = self.t
        n = self.n
        i = self.i
        J = np.array(self.Ni() * (t**i) * ((1-t)**(n-i)))
        return J
    
    def getBezier(self):
        
        for CPTS in range(0,self.nCPTS):
            
            self.b.append(self.basisFunction())
            # Bezier curve calculation
            self.BezierCurve[:,0] = self.basisFunction()*self.x[CPTS] + self.BezierCurve[:,0] # x
            self.BezierCurve[:,1] = self.basisFunction()*self.y[CPTS] + self.BezierCurve[:,1] # y
            self.i += 1
            
        return self.BezierCurve
            
    def getLength(self):
        
        self.curveLength = np.sum(np.sqrt(np.sum((self.BezierCurve[1:]-self.BezierCurve[:-1])**2,axis=1)))
        
        return self.curveLength

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

        self.tag_read_last = np.zeros(2) # for filtering
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

        # rospy.Subscriber(ns + "april_data", Point, self.aprilTagCallback, queue_size=1)
        rospy.Subscriber(ns + "odom", Odometry, self.odometeryCallback, queue_size=1)
        rospy.Subscriber(ns + "april_data_multi",Float32MultiArray, self.multiAprilTagCallback, queue_size=1)
        pub        = rospy.Publisher(ns + "cmd_vel", Twist, queue_size=1)
        pub_data   = rospy.Publisher(ns + "data", Twist, queue_size=1)
        pub_target = rospy.Publisher(ns + "target", Point, queue_size=1)
        pub_l_traj = rospy.Publisher(ns + "l_traj", Float32MultiArray, queue_size=1)

        return pub, pub_data, pub_target, pub_l_traj ,rate
    
    def checkInputs(self):

        while not (len(self.states)>0 and len(self.leader_states)>0):

            rospy.logwarn("waiting for data")

    def lowPass(self, y_current, y_last, lowPassGain = 0.2):
        
        y = lowPassGain*y_last + (1 - lowPassGain)*y_current

        return y

    # def aprilTagCallback(self, data):
    #     '''
    #     robotOdom (+)x = aprilTag (+)z
    #     robotOdom (+)y = aprilTag (-)(x-0.03m)
    #     '''
    #     cam_pose_offset = 0.03
    #     leader_x = data.z
    #     leader_y = -(data.x-cam_pose_offset)
    #     self.leader_state = np.array([leader_x, leader_y])
    #     self.leader_states.append(self.leader_state)

    def multiAprilTagCallback(self, data):
        
        multi_tag  = data.data
        # initialize
        count     = 0
        num_tag   = 3
        tag_space = 5
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
            self.leader_state = self.lowPass(self.leader_state, self.tag_read_last, lowPassGain=0.2)
            self.leader_states.append(self.homoTrans2Global(self.leader_state))

            self.tag_read_last = self.leader_state

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
    
    def homoInvTransMulti2Local(self, states):

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
        twist.linear.x  = ctrl_linear_vel
        twist.linear.y  = 0.0
        twist.linear.z  = 0.0
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
            dx    = self.leader_state[0]
            dy    = self.leader_state[1]
            theta = np.arctan2(dy, dx)

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
        threshold    = 0.10
        check_length = 150 # Might need a larger number if interbot distance is longer
        leader_traj  = np.array(self.leader_states_local)

        while(indx<check_length and len(leader_traj)!=0):
            dist = np.linalg.norm(leader_traj[-indx,:2], ord=2)
            
            if (dist<=threshold or indx==len(leader_traj)):
                # bezier_states = np.asfortranarray([np.append(0., leader_traj[-indx:,0]),
                #                                    np.append(0., leader_traj[-indx:,1])])
                # curve = bezier.Curve(bezier_states, degree=indx)
                bezier = Bezier(np.append(0., leader_traj[-indx:,0]),
                                np.append(0., leader_traj[-indx:,1]))
                curve = bezier.getBezier()

                # evaluate a desired heading angle
                # x = (curve.evaluate(0.05).reshape(2))[0]
                # y = (curve.evaluate(0.05).reshape(2))[1]
                x = curve[40,0]
                y = curve[40,1]
                theta_s = np.arctan2(y, x)

                # e_s = curve.length - distance
                e_s = bezier.getLength() - distance
                target[0] = e_s*np.cos(theta_s)
                target[1] = e_s*np.sin(theta_s)
        
                self.target_status = 2 if dist<=threshold else 3
                print('bezier match indx',indx)

                # record
                target_global  = self.homoTrans2Global(target)
                target_point   = Point()
                target_point.x = target_global[0]
                target_point.y = target_global[1]
                target_point.z = self.target_status
                targetPub.publish(target_point)

                return target, theta_s, bezier.getLength(), True
            
            indx += 1

        # if no feasible fitting points within the 'check_length'
        return None, None, None, False

    def run(self):
        
        freq = 10.0
        cmdVelPub, dataPub, targetPub, lTrajPub, rate = self.initNode(freq)
        self.checkInputs()

        # controller setups
        dt = 1.0/freq
        spacing = 0.4
        ctrl_1  = PD(dt=dt, kp=0.3, kd=1.0, alpha=0.4, dist=spacing)
        ctrl_2  = DSR(dt=dt, kp=0.3, kd=1.0, alpha=0.4, beta=1.0, dist=spacing)


        while not rospy.is_shutdown():

            self.leader_states_local = self.homoInvTransMulti2Local(self.leader_states)
            goal, theta_s, curvelength_s, getTarget = self.getBezierTarget(targetPub, distance=spacing)
            getTarget = False

            if not getTarget:
                rospy.logwarn('bezier fail')
                goal = self.getUnitCircleTarget(targetPub, distance=spacing)
                u = ctrl_1.pd(self.velocity, goal, dataPub)
            else:
                u = ctrl_1.pd_s(self.velocity, curvelength_s, theta_s, dataPub)
                #u = ctrl_2.dsr(curvelength_s, theta_s, self.velocity, self.state[2], self.leader_state)

            print("self.target_status", self.target_status)
            # print('leader state',self.leader_state,'goal ', goal)
            # print('----------------------------------------------')

            self.pubLeaderTraj(lTrajPub)
            self.pubCmdVel(cmdVelPub, u[0], u[1])
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':

    Tracking_node = tracking_node()
    Tracking_node.run()


