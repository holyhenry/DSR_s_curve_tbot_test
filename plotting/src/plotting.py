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
        
        self.leader_state = np.zeros(3)
        self.follower_state = np.zeros(3)

        self.leader_states = []
        self.follower_states = []

    def initNode(self, freq):

        rospy.init_node('plotting_node')
        rate = rospy.Rate(int(freq))
        leader_ns = "/tbot165/"
        follower_ns = "/tbot199/"

        rospy.Subscriber(leader_ns + "odom", Odometry, self.leaderOdomCallback, queue_size=1)
        rospy.Subscriber(follower_ns + "odom", Odometry, self.followerOdomCallback, queue_size=1)

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

    def plot(self, fig):

        distance = 0.35

        if len(self.follower_states) != 0 and len(self.leader_states) != 0:
            f_len = len(self.follower_states)
            l_len = len(self.leader_states)
            plt.cla()
            plt.plot(np.array(self.follower_states)[0:f_len,0], np.array(self.follower_states)[0:f_len,1],marker='.',color='green')
            plt.plot(np.array(self.leader_states)[0:l_len,0]+distance, np.array(self.leader_states)[0:l_len,1],marker='.',color='red')
            plt.plot(np.array(self.follower_states)[-1,0], np.array(self.follower_states)[-1,1],marker='o',color='green',label="follower")
            plt.plot(np.array(self.leader_states)[-1,0]+distance, np.array(self.leader_states)[-1,1],marker='o',color='red',label="leader")
            plt.axis('equal')
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.xlim(-1,3)
            plt.ylim(-4,4)
            plt.legend()
            plt.grid()
            
            fig.canvas.flush_events()
            time.sleep(0.1)
        
            # print(np.array(self.follower_states)[:,0].shape)

    def run(self):

        freq = 10.0
        self.initNode(freq)

        # plotting setup
        plt.ion()
        fig = plt.figure(figsize=(10,8))
        # fig.show()

        while not rospy.is_shutdown():

            self.plot(fig)


if __name__ == '__main__':

    Plotting_node = plotting_node()
    Plotting_node.run()