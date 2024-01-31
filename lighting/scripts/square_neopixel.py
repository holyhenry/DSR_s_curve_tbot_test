#!/usr/bin/env python
import board
import neopixel
import rospy
import numpy as np
from geometry_msgs.msg import Twist

ns = rospy.get_namespace()

bright = rospy.get_param(ns + "/lighting/brightness")
pixels = neopixel.NeoPixel(board.D10, 64, brightness=bright)
# pixels = neopixel.NeoPixel(board.D18, 64, brightness=bright)
        
def angleDataCallback(msg):

    angle = np.abs((msg.angular.z)*180/np.pi)
    smile_face   = [3,13,17,21,22,31,32,41,42,46,50,60]
    cry_face     = [0,14,18,21,22,28,35,41,42,45,49,63]
    neutral_face = [2,13,18,21,22,29,34,41,42,45,50,61]

    if angle < 45:
        color = (0, 255, 0) # green
        for i in smile_face:
            pixels[i] = color

    elif 45 <= angle and angle < 60:
        color = (255, 69, 0) # yellow
        for i in neutral_face:
            pixels[i] = color

    else:
        color = (255, 0, 0) # red
        for i in cry_face:
            pixels[i] = color
    
    # pixels.fill(color) 
     

def listener():
    
    rospy.init_node('lighting', anonymous=True)
    rospy.Subscriber(ns + "data", Twist, angleDataCallback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    
    listener()
    if rospy.is_shutdown:
        pixels.fill((0, 0, 0))