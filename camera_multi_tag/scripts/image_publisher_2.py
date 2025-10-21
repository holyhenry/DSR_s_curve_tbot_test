import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np
import apriltag

import rospy
from sensor_msgs.msg import Image
from common_msgs.msg import Float32ArrayStamped

rospy.init_node('image_publisher')
r             = rospy.Rate(30) # 10hz
LP_GAIN       = rospy.get_param('~tag_lp_gain', 0.2)
ns            = rospy.get_namespace()
image_pub     = rospy.Publisher(ns + 'camera/color/image_raw', Image, queue_size=2)
detection_pub = rospy.Publisher(ns + 'tag_detections', Float32ArrayStamped, queue_size=10)

# try to disable depth information, need to verify
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.disable_stream(rs.stream.depth)

cap = cv2.VideoCapture(4)
br  = CvBridge()

# Tag size in meters
tag_size = 0.04

# Define the 3D coordinates for the tag corners:
# Assuming the detected order is [top-left, top-right, bottom-right, bottom-left]
object_points = np.array([
                          [-tag_size/2,  tag_size/2, 0],
                          [ tag_size/2,  tag_size/2, 0],
                          [ tag_size/2, -tag_size/2, 0],
                          [-tag_size/2, -tag_size/2, 0]
                         ], dtype=np.float32)

# Example intrinsic camera matrix (3x3)
camera_matrix = np.array([
                          [610.3250122070312,  0, 331.69219970703125],
                          [ 0, 610.5338134765625, 239.29869079589844],
                          [ 0,                 0,                  1]
                         ], dtype=np.float32)

# Distortion coefficients (k1, k2, p1, p2, k3)
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)   

# Initialize AprilTag detector
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

# Help functions
tvec_lp_state = {}  # tag_id -> np.array([tx,ty,tz], float)

def _low_pass(x, x_last, lp_gain = 0.2):
        return lp_gain * x + (1 - lp_gain) * x_last

while not rospy.is_shutdown():

    # Allow live filtr tuning
    LP_GAIN = rospy.get_param('~tag_lp_gain', LP_GAIN)

    ret, frame = cap.read()
    if ret:
        # Convert to grayscale for AprilTag detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = detector.detect(gray_frame)

        # Prepare detection data
        W = 5 
        detection_data = [0 for i in range(W*len(detections))]

        for i, detection in enumerate(detections):
            # Convert detected corners to proper format
            image_points = np.array(detection.corners, dtype=np.float32)
            
            # Compute pose of the detected tag
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            if not success:
                rospy.logwarn("Pose estimation failed for tag ID %d", detection.tag_id)

            # Convert the rotation vector to a rotation matrix.
            R, _ = cv2.Rodrigues(rvec)

            # Compute the intermediary value to handle singularities (gimbal lock)
            sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

            # Check if the matrix is close to a singular configuration
            singular = sy < 1e-6

            # Convert R to raw, pitch, yaw w.r.t actual robot frame
            if not singular:
                #roll = np.arctan2(R[2, 1], R[2, 2])
                pitch = np.arctan2(-R[2, 0], sy)  # Rotation about y axis.
                #yaw = np.arctan2(R[1, 0], R[0, 0])
            else:  # Gimbal lock: We set yaw to zero and compute roll differently.
                #roll = np.arctan2(-R[1, 2], R[1, 1])
                pitch = np.arctan2(-R[2, 0], sy)
                #yaw = 0

            # Low-pass filter tvec per tag
            tid = int(detection.tag_id)
            tvec = tvec = tvec.flatten().astype(float)
            if tid in tvec_lp_state:
                tvec_f = _low_pass(tvec, tvec_lp_state[tid], lp_gain=float(LP_GAIN))
            else:
                tvec_f = tvec
            tvec_lp_state[tid] = tvec_f
            
            # Fill payload with FILTERED translation (x,y,z) + pitch
            base = i*W
            detection_data[base + 0] = float(tid)
            detection_data[base + 1] = float(tvec_f[0])  # tag_x: actual robot frame - left(-) & right(+)
            detection_data[base + 2] = float(tvec_f[1])  # tag_y: actual robot frame - up(-) & down(+)
            detection_data[base + 3] = float(tvec_f[2])  # tag_z: actual robot frame - foreward(+) & backward(-)
            detection_data[base + 4] = float(pitch)      # tag_yaw: rotate about tag_y 

        # print(detection_data)

        # Publishers
        msg = Float32ArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_frame"

        msg.data.data = detection_data
        detection_pub.publish(msg)
        # image_pub.publish(br.cv2_to_imgmsg(frame))

    r.sleep()
