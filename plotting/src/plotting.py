import time
import numpy as np
import matplotlib.pyplot as plt

import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry

class plotting_node:

    def __init__(self) -> None:
        
        self.cum_angle = 0.0
        self.leader_state = np.zeros(3)
        self.follower_state = np.zeros(3)
        self.leader_state_tag = np.zeros(3)

        # odom (global)
        self.leader_states = []
        self.follower_states = []
        # tag info global
        self.leader_states_tag = []
        # tag info local
        self.leader_states_local = []


    def initNode(self, freq):

        rospy.init_node('plotting_node')
        rate = rospy.Rate(int(freq))
        leader_ns = "/tbot165/"
        follower_ns = "/tbot199/"

        rospy.Subscriber(leader_ns + "odom", Odometry, self.leaderOdomCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "odom", Odometry, self.followerOdomCallback, queue_size=1)
        # rospy.Subscriber(follower_ns + "april_data_multi",Float32MultiArray, self.multiAprilTagCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "l_traj", Float32MultiArray, self.leaderInferedTrajCallback, queue_size=1)
    
    def multiAprilTagCallback(self, data):
        
        count     = 0
        num_tag   = 3
        tag_space = 5
        multi_tag = data.data
        leader_x  = 0.0
        leader_y  = 0.0
        follower_indx   = 0
        cam_pose_offset = 0.03

        for i in range(len(multi_tag)):
            if (i%tag_space == 0 and (multi_tag[i]//num_tag == follower_indx)):
                count += 1
                x   = multi_tag[i+3]
                y   = -(multi_tag[i+1]-cam_pose_offset)
                phi = multi_tag[i+4]
                id  = multi_tag[i]
                infered_x, infered_y = self.transformTag2Middle(x,y,phi,id)
                leader_x += infered_x
                leader_y += infered_y
        
        if (count != 0):
            
            leader_x /= count
            leader_y /= count
            self.leader_state_tag = self.homoTrans2Global(np.array([leader_x, leader_y]))
            self.leader_states_tag.append(self.leader_state_tag)

    def homoTrans2Global(self, state):

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
    
    def homoInvTrans2Local(self, states):

        del_theta = self.follower_state[2]
        del_pos   = self.follower_state[:2]

        rot_matrix = np.array([[np.cos(del_theta), -np.sin(del_theta)],
                            [np.sin(del_theta), np.cos(del_theta)]])
        trans_vec = np.atleast_2d(del_pos).T
        T = np.hstack([rot_matrix, trans_vec])
        T = np.vstack([T, np.array([0.,0.,1.])])

        ones = np.atleast_2d(np.ones(np.array(states).shape[0])).T
        new_states = np.hstack([states, ones])
        new_states = np.einsum('ij,kj->ki', np.linalg.inv(T), new_states)
    
        return new_states[:,:2]

    def transformTag2Middle(self, x, y, alpha, id, num_tag=3, d=0.038):

        if (id%num_tag == 0):
            x1_hat = x + d*np.cos(-np.pi/2+alpha+0.2616)
            y1_hat = y + d*np.sin(-np.pi/2+alpha+0.2616)
            return x1_hat, y1_hat
        
        if (id%num_tag == 2):
            x1_hat = x + d*np.cos(np.pi/2+alpha-0.2616)
            y1_hat = y + d*np.sin(np.pi/2+alpha-0.2616)
            return x1_hat, y1_hat
        
        else:
            return x, y

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
        
        self.leader_states_tag = np.reshape(list(data.data),(-1,2))
        print(self.leader_states_tag)
        
    def plot(self, fig):

        distance = 0.35

        if len(self.follower_states) != 0 and len(self.leader_states) != 0:

            f_len = len(self.follower_states)
            l_len = len(self.leader_states)
            il_len = len(self.leader_states_tag)
            # self.leader_states_local = self.homoInvTrans2Local(self.leader_states_tag)

            plt.cla()
            # plot odom trajectory
            plt.plot(np.array(self.follower_states)[0:f_len,0], np.array(self.follower_states)[0:f_len,1],marker='.',color='green')
            plt.plot(np.array(self.leader_states)[0:l_len,0]+distance, np.array(self.leader_states)[0:l_len,1],marker='.',color='red')
            # plot tag-infered trajecotry
            plt.plot(np.array(self.leader_states_tag)[0:il_len,0], np.array(self.leader_states_tag)[0:il_len,1],marker='.',color='orange')
            #plt.plot(np.array(self.leader_states_local)[0:il_len,0], np.array(self.leader_states_local)[0:il_len,1],marker='.',color='black')

            # plot current state
            plt.plot(np.array(self.follower_states)[-1,0], np.array(self.follower_states)[-1,1],marker='o',color='green',label="follower")
            plt.plot(np.array(self.leader_states)[-1,0]+distance, np.array(self.leader_states)[-1,1],marker='o',color='red',label="leader")
            plt.plot(np.array(self.leader_states_tag)[-1,0], np.array(self.leader_states_tag)[-1,1],marker='o',color='orange',label="infered")

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

        freq = 20.0
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