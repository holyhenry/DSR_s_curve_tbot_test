#!/usr/bin/env python
import board
import neopixel
import rospy
import numpy as np
from geometry_msgs.msg import Twist

class lighting_node:
    
    def __init__(self) -> None:

        self.bright = 0
        self.angle = 0.0

        self.mode = 0
        self.mode_last = 0

    def initNode(self):

        rospy.init_node('lighting', anonymous=True)
        ns = rospy.get_namespace()

        self.bright = rospy.get_param(ns + "/lighting/brightness")
        self.pixels = neopixel.NeoPixel(board.D10, 64, brightness=self.bright)
        # pixels = neopixel.NeoPixel(board.D18, 64, brightness=bright)

        rospy.Subscriber(ns + "data", Twist, self.angleDataCallback, queue_size=1)

    def angleDataCallback(self, msg):

        self.angle = np.abs((msg.angular.z)*180/np.pi)

    def run(self):

        self.initNode()

        smile_face   = [3,13,17,21,22,31,32,41,42,46,50,60]
        cry_face     = [0,14,18,21,22,28,35,41,42,45,49,63]
        neutral_face = [2,13,18,21,22,29,34,41,42,45,50,61]

        while not rospy.is_shutdown():
            if self.angle < 45:
                self.mode = 1
                if self.mode != self.mode_last:
                    self.pixels.fill((0,0,0))
                    print('mode change!') 
                for i in smile_face:
                    self.pixels[i] = (0, 255, 0) # green

            elif 45 <= self.angle and self.angle < 60:
                self.mode = 2
                if self.mode != self.mode_last:
                    self.pixels.fill((0,0,0))
                    print('mode change!') 
                for i in neutral_face:
                    self.pixels[i] = (255, 69, 0) # yellow

            else:
                self.mode = 3
                if self.mode != self.mode_last:
                    self.pixels.fill((0,0,0))
                    print('mode change!') 
                for i in cry_face:
                    self.pixels[i] = (255, 0, 0) # red

        self.mode_last = self.mode

        if rospy.is_shutdown:
            self.pixels.fill((0, 0, 0))

        rospy.spin()

if __name__ == '__main__':

    Lighting_node = lighting_node()
    Lighting_node.run()
