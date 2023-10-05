import rospy
import cv2
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('image_publisher')
r   = rospy.Rate(20) # 10hz

ns  = rospy.get_namespace()
pub = rospy.Publisher(ns + '/camera/color/image_raw', Image, queue_size=2)

# try to disable depth information, need to verify
config = rs.config()
config.disable_stream(rs.stream.depth)

cap = cv2.VideoCapture(4)
br  = CvBridge()

while not rospy.is_shutdown():
    #print('rs~ ',rs.stream.accel)
    ret, frame = cap.read()
    if ret == True:
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        pub.publish(br.cv2_to_imgmsg(frame))

    r.sleep()
