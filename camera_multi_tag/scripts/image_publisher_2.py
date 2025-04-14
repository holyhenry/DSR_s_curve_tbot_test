import rospy
import cv2
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import apriltag

rospy.init_node('image_publisher')
r   = rospy.Rate(30) # 10hz

ns  = rospy.get_namespace()
pub = rospy.Publisher(ns + 'camera/color/image_raw', Image, queue_size=2)

# try to disable depth information, need to verify
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.disable_stream(rs.stream.depth)

cap = cv2.VideoCapture(4)
br  = CvBridge()

# Tag size in meters
tag_size = 0.045

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

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        # Convert to grayscale for AprilTag detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = detector.detect(gray_frame)

        # Prepare detection data
        detection_data = [0 for i in range(5*len(detections))]

        for detection in detections:
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

            detection_data.append([
                detection.tag_id,
                tvec[0],  # tag_x: actual robot frame - left(-) & right(+)
                tvec[1],  # tag_y: actual robot frame - up(-) & down(+)
                tvec[2],  # tag_z: actual robot frame - foreward(+) & backward(-)
                pitch     # tag_yaw: rotate about tag_y 
            ])

        print(len(detections))

        # Publish the image.
        pub.publish(br.cv2_to_imgmsg(frame))

    r.sleep()
