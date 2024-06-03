import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector

# Configure the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the RealSense pipeline
pipeline.start(config)

# Create an AprilTag detector
detector = Detector(families='tag36h11',
                    nthreads=4,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the color image to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the image
        tags = detector.detect(gray_image)

        # Draw detections on the color image
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_image, 
                         tuple(tag.corners[idx-1, :].astype(int)), 
                         tuple(tag.corners[idx, :].astype(int)), 
                         (0, 255, 0), 2)
            cv2.putText(color_image, str(tag.tag_id),
                        (tag.corners[0, 0].astype(int), tag.corners[0, 1].astype(int) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with detections
        cv2.imshow('AprilTag Detection', color_image)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and close the window
    pipeline.stop()
    cv2.destroyAllWindows()
