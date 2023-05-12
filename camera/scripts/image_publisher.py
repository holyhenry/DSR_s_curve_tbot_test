import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=2)
rospy.init_node('image_publisher')
r = rospy.Rate(50) # 10hz

cap = cv2.VideoCapture(4)
br = CvBridge()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret == True:
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        pub.publish(br.cv2_to_imgmsg(frame))

    r.sleep()