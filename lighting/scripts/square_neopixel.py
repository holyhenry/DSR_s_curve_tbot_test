#!/usr/bin/env python
import board
import neopixel
import rospy
import numpy as np
from geometry_msgs.msg import Twist

angle = 0

mode = 0
mode_last = 0

ns = rospy.get_namespace()
bright = rospy.get_param(ns + "/lighting/brightness")
pixels = neopixel.NeoPixel(board.D10, 64, brightness=bright)
# pixels = neopixel.NeoPixel(board.D18, 64, brightness=bright)

def angleDataCallback(msg):

    angle = np.abs((msg.angular.z)*180/np.pi)

def runLights():

    smile_face   = [3,13,17,21,22,31,32,41,42,46,50,60]
    cry_face     = [0,14,18,21,22,28,35,41,42,45,49,63]
    neutral_face = [2,13,18,21,22,29,34,41,42,45,50,61]

    if angle < 45:
        mode = 1
        if mode != mode_last:
            pixels.fill((0,0,0))
            print('mode change!') 
        for i in smile_face:
            pixels[i] = (0, 255, 0) # green

    elif 45 <= angle and angle < 60:
        mode = 2
        if mode != mode_last:
            pixels.fill((0,0,0))
            print('mode change!') 
        for i in neutral_face:
            pixels[i] = (255, 69, 0) # yellow

    else:
        mode = 3
        if mode != mode_last:
            pixels.fill((0,0,0))
            print('mode change!') 
        for i in cry_face:
            pixels[i] = (255, 0, 0) # red

    mode_last = mode

def listener():

    rospy.init_node('lighting', anonymous=True)
    rospy.Subscriber(ns + "data", Twist, angleDataCallback, queue_size=1)
    runLights()
    rospy.spin()

if __name__ == '__main__':

    listener()
    if rospy.is_shutdown:
        pixels.fill((0, 0, 0))