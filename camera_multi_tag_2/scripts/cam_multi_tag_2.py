import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector

# ros package
import rospy
from std_msgs.msg import Float32MultiArray

np.set_printoptions(precision=4)

class cam_multi_tag_node_2:

    def __init__(self):
        
        # Apriltag params
        self.TAG_SIZE = 0.04 # meter
        self.at_detector = Detector(families='tag36h11',
                                    nthreads=1,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        
        cv2.destroyAllWindows()
        # self.connect_device() # rasp pi currently has issue with pyrealsense 
        
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config   = rs.config()

        # Start streaming
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg = self.pipeline.start(self.config)
        self.cam_param = self.get_rs_param(cfg)
        print("Starting to stream")

    def initNode(self, freq):

        rospy.init_node('camera_multi_tag_2')
        rate = rospy.Rate(int(freq))
        ns   = rospy.get_namespace()

        pub = rospy.Publisher(ns + "april_data_multi", Float32MultiArray, queue_size=3)

        return pub, rate

    def connect_device(self):

        ctx = rs.context()
        serials = []
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()

        if len(ctx.devices) > 0:
            for dev in ctx.devices:
                print('Found device:', dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))
                serials.append(dev.get_info(rs.camera_info.serial_number))
        else:
            print("No Intel Device connected")

    def get_rs_param(self, cfg):

        profile = cfg.get_stream(rs.stream.color)
        intr = profile.as_video_stream_profile().get_intrinsics()

        return [intr.fx, intr.fy, intr.ppx, intr.ppy]

    def run(self, freq):

        detectPub, rate = self.initNode(freq)
        cap = cv2.VideoCapture(2)

        while not rospy.is_shutdown():

            # Wait for a coherent pair of frames: depth and color
            ret, frame = cap.read()
            # frames = self.pipeline.wait_for_frames()
            # color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            # rgb = np.asanyarray(color_frame.get_data())
            # rgb = cv2.GaussianBlur(rgb,(15,15),sigmaX=2.5, sigmaY=2.5)

            if ret:
                
                rgb = frame

                # Tag detection
                gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
                results = self.at_detector.detect(gray, estimate_tag_pose=True, tag_size=self.TAG_SIZE, 
                                                    camera_params=self.cam_param)
                
                # Convert tvec rvec to tag data
                n_data = 5
                data = np.zeros(len(results)*n_data)
                for i, result in enumerate(results):

                    R = result.pose_R
                    euler_y = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2+R[2,2]**2))

                    data[n_data*i]     = result.tag_id
                    data[n_data*i + 1] = result.pose_t[0,0] # robot frame x - left(-) & right(+)
                    data[n_data*i + 2] = result.pose_t[1,0] # robot frame y - up(-) & down(+)
                    data[n_data*i + 3] = result.pose_t[2,0] # robot frame z - foreward(+) & backward(-)
                    data[n_data*i + 4] = euler_y
                    # print(euler_y*180/np.pi)

                    # cv2.imshow('Raw', rgb)
                    # cv2.waitKey(1)

                tag_data = Float32MultiArray()
                tag_data.data = data
                detectPub.publish(tag_data)

            rate.sleep()
        
        rospy.spin()


if __name__ == '__main__':

    cam_detect = cam_multi_tag_node_2()

    try:
        cam_detect.run(freq=30)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        cam_detect.pipeline.stop()

                    