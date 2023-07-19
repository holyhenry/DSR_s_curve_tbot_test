import time
import bezier
import numpy as np
import matplotlib.pyplot as plt

import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry

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

class plotting_node:

    def __init__(self) -> None:
        
        self.cum_angle = 0.0
        self.leader_state = np.zeros(3)
        self.follower_state = np.zeros(3)
        self.leader_state_tag = np.zeros(2)

        # odom (global)
        self.leader_states = []
        self.follower_states = []
        # tag info global
        self.leader_states_tag = []
        self.leader_states_tag_frombag = []
        # tag info local
        self.leader_states_local = []

        # from rosbag
        self.tracking_target = np.zeros(2)
        self.tracking_target_type = -1
        
        self.tag_read_last = np.zeros(2) # for filtering
        self.target_status = -1

        # Bezier curve
        self.bezier_curve = np.zeros((1,2))


    def initNode(self, freq):

        rospy.init_node('plotting_node')
        rate = rospy.Rate(int(freq))
        leader_ns = "/tbot165/"
        follower_ns = "/tbot199/"

        rospy.Subscriber(leader_ns + "odom", Odometry, self.leaderOdomCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "odom", Odometry, self.followerOdomCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "april_data_multi",Float32MultiArray, self.multiAprilTagCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "l_traj", Float32MultiArray, self.leaderInferedTrajCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "target", Point, self.trackingTargetCallback,queue_size=1)

    def lowPass(self, y_current, y_last, lowPassGain=0.5):

        return lowPassGain*y_last + (1 - lowPassGain)*y_current

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
            self.leader_state_tag = self.homoTrans2BotCenter(np.array([tag_x,tag_y,tag_phi]))
            self.leader_state_tag = self.lowPass(self.leader_state_tag, self.tag_read_last, lowPassGain=0.0)
            self.tag_read_last = self.leader_state_tag

            self.leader_states_tag.append(self.homoTrans2Global(self.leader_state_tag))
    
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
        '''
        single transform from tag pos to leader's geometric center
        '''
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
        '''
        single transform from local to global
        '''
        del_theta = self.follower_state[2]
        del_pos   = self.follower_state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                               [np.sin(del_theta), np.cos(del_theta)]]) 
        trans_vec  = np.atleast_2d(del_pos).T   
        T = np.hstack([rot_matrix, trans_vec])
        T = np.vstack([T, np.array([0.,0.,1.])])   

        homo_state = np.hstack([state, 1.])
        new_state = (T@homo_state)[:2]

        return new_state

    def homoTransMulti2Global(self, states):
        '''
        multi transform from local to global
        '''
        del_theta = self.follower_state[2]
        del_pos   = self.follower_state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                               [np.sin(del_theta), np.cos(del_theta)]]) 
        trans_vec  = np.atleast_2d(del_pos).T   
        T = np.vstack([np.hstack([rot_matrix, trans_vec]), np.array([0.,0.,1.])])   

        ones   = np.atleast_2d(np.ones(np.array(states).shape[0])).T
        try:
            homo_states = np.hstack([states, ones])
        except:
            print('2Global: size mismatch!')
            ones = np.atleast_2d(np.ones(np.array(states).shape[0])).T
            homo_states = np.hstack([states, ones])
        new_states = np.einsum('ij,kj->ki', T, homo_states)[:,:2]

        return new_states
    
    def homoInvTransMulti2Local(self, states):
        '''
        inverse multi transform from global to local
        '''
        del_theta = self.follower_state[2]
        del_pos   = self.follower_state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                            [np.sin(del_theta), np.cos(del_theta)]])
        trans_vec = np.atleast_2d(del_pos).T
        T = np.vstack([np.hstack([rot_matrix, trans_vec]), np.array([0.,0.,1.])])

        ones = np.atleast_2d(np.ones(np.array(states).shape[0])).T
        try:
            homo_states = np.hstack([states, ones])
        except:
            print('2Local: size mismatch!')
            ones = np.atleast_2d(np.ones(np.array(states).shape[0])).T
            homo_states = np.hstack([states, ones])
        new_states = np.einsum('ij,kj->ki', np.linalg.inv(T), homo_states)[:,:2]
    
        return new_states

    def leaderOdomCallback(self, data):

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        (_, _, yaw) = transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])

        self.leader_state = np.array([x, y, yaw])
        self.leader_states.append(self.leader_state)

    def followerOdomCallback(self, data):

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        (_, _, yaw) = transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])

        self.follower_state = np.array([x, y, yaw])
        self.follower_states.append(self.follower_state)

    def leaderInferedTrajCallback(self, data):
        
        self.leader_states_tag_frombag = np.reshape(list(data.data),(-1,2))
        self.leader_states_tag_frombag = self.homoTransMulti2Global(self.leader_states_tag_frombag)

    def trackingTargetCallback(self, data):

        self.tracking_target[0] = data.x
        self.tracking_target[1] = data.y
        self.tracking_target_type = data.z

    def getUnitCircleTarget(self, distance):

        target = np.zeros(2)
        find_target = False
        leader_traj = np.array(self.leader_states_local)

        for i in range(len(leader_traj)-1, -1, -1):
            travel_length = leader_traj[i][:]-self.leader_state_tag[:]

            if np.linalg.norm(travel_length, ord=2)>=distance:
                target = leader_traj[i][:]
                self.target_status = 1
                find_target = True
                break

        if not find_target:
            dx = self.leader_state_tag[0]
            dy = self.leader_state_tag[1]
            theta = np.arctan2(dy, dx)
            target[0] = self.leader_state_tag[0] - distance*np.cos(theta)
            target[1] = self.leader_state_tag[1] - distance*np.sin(theta)
            self.target_status = 0

        return target
    
    def getBezierTarget(self, distance):
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

                return target, True
            
            indx += 1

        # no feasible fitting point within the 'check_length'
        return None, False

    def getBezierTarget_new(self, distance):
        '''
        return: s_l(t)-s_f(t) and desired heading
        '''
        indx   = 1
        target = np.zeros(2)
        threshold    = 0.10
        check_length = 150 # Might need a larger number if interbot distance is longer
        leader_traj  = np.array(self.leader_states_local)

        while(indx<check_length and len(leader_traj)!=0):
            # dist = np.linalg.norm(self.state[:2]-leader_traj[-indx,:2], ord=2)
            dist = np.linalg.norm(leader_traj[-indx,:2], ord=2)
            print('indx:',indx,' dist:',dist)
            
            if (dist<=threshold or indx==len(leader_traj)):
                # print('new bezier :), match indx:',indx)
                # bezier_states = np.asfortranarray([np.append(0., leader_traj[-indx:,0]),
                #                                    np.append(0., leader_traj[-indx:,1])])
                # curve = bezier.Curve(bezier_states, degree=indx)
                bezier = Bezier(np.append(0., leader_traj[-indx:,0]),
                                np.append(0., leader_traj[-indx:,1]))
                curve = bezier.getBezier()

                # evaluate a desired heading angle
                # x = (curve.evaluate(0.05).reshape(2))[0]
                # y = (curve.evaluate(0.05).reshape(2))[1]
                x = curve[35,0]
                y = curve[35,1]
                theta_s = np.arctan2(y, x)

                # e_s = curve.length - distance
                e_s = bezier.getLength() - distance
                target[0] = e_s*np.cos(theta_s)
                target[1] = e_s*np.sin(theta_s)
                
                self.target_status = 2 if dist<=threshold else 3

                return target, curve, True
            
            indx += 1

        # no feasible fitting point within the 'check_length'
        return None, None, False

    def plot(self, fig):

        distance = 0.50
        
        if (len(self.follower_states)>0 and len(self.leader_states)>0 and len(self.leader_states_tag)>0):

            f_len   = len(self.follower_states)
            l_len   = len(self.leader_states)
            il_len  = len(self.leader_states_tag_frombag)
            ill_len = len(self.leader_states_tag)

            self.leader_states_local = self.homoInvTransMulti2Local(self.leader_states_tag)
            goal, self.bezier_curve, getGoal = self.getBezierTarget_new(distance=distance)
            # if not getGoal:
            #     rospy.logwarn("bezier fail!!")
            #     goal = self.getUnitCircleTarget(distance=distance)

            if self.bezier_curve is not None:
                self.bezier_curve = self.homoTransMulti2Global(self.bezier_curve) # just for view
                b_len = len(self.bezier_curve)
            # print('target_status:', self.target_status)

            print('target_status from bag: ',self.tracking_target_type)

            plt.cla()
            '''plot odom trajectory'''
            plt.plot(np.array(self.follower_states)[0:f_len,0], np.array(self.follower_states)[0:f_len,1],marker='.',color='green')
            plt.plot(np.array(self.leader_states)[0:l_len,0]+distance, np.array(self.leader_states)[0:l_len,1],marker='.',color='red')
            '''plot current state'''
            plt.plot(np.array(self.leader_states)[-1,0]+distance, np.array(self.leader_states)[-1,1],marker='o',color='red',label="leader odom")
            plt.plot(np.array(self.follower_states)[-1,0], np.array(self.follower_states)[-1,1],marker='o',color='green',label="follower odom")
            # plt.plot(np.array(self.leader_states_tag_frombag)[-1,0], np.array(self.leader_states_tag_frombag)[-1,1],marker='o',color='purple',label="local infered")
            # plt.plot(np.array(self.leader_states_tag_frombag)[0:il_len,0], np.array(self.leader_states_tag_frombag)[0:il_len,1],marker='.',color='purple')
            '''plot tag-infered trajecotry''' 
            plt.plot(np.array(self.leader_states_tag)[-1,0], np.array(self.leader_states_tag)[-1,1],marker='o',color='orange')
            plt.plot(np.array(self.leader_states_tag)[0:ill_len,0], np.array(self.leader_states_tag)[0:ill_len,1],marker='.',color='orange',label="tag infered (g=0)")
            '''plot bezier curve'''
            if self.bezier_curve is not None:
                plt.plot(self.bezier_curve[0:b_len,0],self.bezier_curve[0:b_len,1],marker='.',color='lightcoral',label='computed bezier curve')
            '''plot tracking target'''
            plt.plot(self.tracking_target[0],self.tracking_target[1],marker='x',color='black',label='recorded target') 
            # plt.plot(goal_global[0],goal_global[1],marker='x',color='red',label='computed target')

            plt.axis('equal')
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.xlim(-1,3)
            plt.ylim(-3,1)
            plt.legend()
            plt.grid()
        
            fig.canvas.flush_events()
            time.sleep(0.05)

    def run(self):

        freq = 10.0
        self.initNode(freq)

        # plotting setup
        plt.ion()
        fig = plt.figure(figsize=(10,8))
        fig.show()

        while not rospy.is_shutdown():

            self.plot(fig)


if __name__ == '__main__':

    Plotting_node = plotting_node()
    Plotting_node.run()