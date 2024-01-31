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

def callback(msg):
    if msg.data == "red":
        color = (255,0,0)
    
    elif msg.data == "green":
        color = (0,255,0)

    elif msg.data == "blue":
        color = (0,0,255)

    elif msg.data == "yellow":
        color = (255,69,0)

    else:
        color = (0,0,0)
        pixels.fill((0, 0, 0))

    try:
        # Square shape
        for i in range(0, 16):
            pixels[i] = color

        for i in range(48,64):
            pixels[i] = color

        for i in range(2,6):
            pixels[0 + (8*i)] = color
            pixels[1 + (8*i)] = color
            pixels[6 + (8*i)] = color
            pixels[7 + (8*i)] = color
    except:
        print("error in neopixel")
        
   # print("subscribing")
   # time.sleep(0.5)
        
def angleDataCallback(msg):

    angle = np.abs((msg.angular.z)*180/np.pi)

    if angle < 45:
        color = (0, 255, 0) # green

    elif 45 <= angle and angle < 60:
        color = (255, 69, 0) # yellow

    else:
        color = (255, 0, 0) # red
    
    pixels.fill(color) 

def listener():
    
    rospy.init_node('lighting', anonymous=True)
    rospy.Subscriber(ns + "/data", Twist, angleDataCallback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
    if rospy.is_shutdown:
        pixels.fill((0, 0, 0))