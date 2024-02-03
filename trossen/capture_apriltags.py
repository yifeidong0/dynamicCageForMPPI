# """
# Currently only detects AprilTags and draws them on the image (in the image frame).
# Next steps:
# - Get the pose of the AprilTags in the camera frame
# """
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import apriltag

# # Initialize AprilTag detector
# detector = apriltag.Detector()

# # Assuming the IDs for W, X, and Y are 27, 28, and 29, respectively
# # Initialize variables to store the first detection of W
# w_pose = None

# # Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# # Start streaming
# pipeline.start(config)

# try:
#     while True:
#         # Wait for a coherent pair of frames: depth and color
#         frames = pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         # depth_frame = frames.get_depth_frame()
#         if not color_frame:
#             continue

#         # Convert images to numpy arrays
#         color_image = np.asanyarray(color_frame.get_data())

#         # Inside your main loop, after capturing the color_image
#         gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
#         detections = detector.detect(gray)
#         for detection in detections:
#             # Draw detected AprilTag
#             for idx, point in enumerate(detection.corners):
#                 pt = tuple(point.astype(int))
#                 cv2.circle(color_image, pt, 5, (0, 255, 0), thickness=-1)
#             cv2.putText(color_image, str(detection.tag_id), tuple(detection.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
#         # Show images
#         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('RealSense', color_image)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
# finally:
#     # Stop streaming
#     pipeline.stop()



import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag

# Initialize the detector
detector = apriltag.Detector()

# Camera intrinsic parameters
# Replace these values with your camera's specific parameters
intrinsics = {
    "fx": 640,  # Focal length in x
    "fy": 480,  # Focal length in y
    "cx": 320,  # Principal point x
    "cy": 240,  # Principal point y
}
camera_matrix = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                          [0, intrinsics["fy"], intrinsics["cy"]],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Tag size (meters)
tag_size = 0.16  # Adjust based on your actual tag size
axis_length = 0.1  # Meters

# Configure and start the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        for detection in detections:
            # Assuming square tags, for simplicity
            tag_corners_2d = np.array(detection.corners, dtype=np.float32)
            tag_corners_3d = np.array([[-tag_size / 2, -tag_size / 2, 0],
                                       [ tag_size / 2, -tag_size / 2, 0],
                                       [ tag_size / 2,  tag_size / 2, 0],
                                       [-tag_size / 2,  tag_size / 2, 0]])

            # Use solvePnP to get the tag pose
            retval, rvec, tvec = cv2.solvePnP(tag_corners_3d, tag_corners_2d, camera_matrix, dist_coeffs)

            # Draw the detected tag
            for idx, point in enumerate(detection.corners):
                pt = tuple(point.astype(int))
                cv2.circle(color_image, pt, 5, (0, 255, 0), thickness=-1)
            cv2.putText(color_image, str(detection.tag_id), tuple(detection.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Optionally, draw the pose on the image
            # Directly use cv2.drawFrameAxes to draw the axes on the color image
            print("camera_matrix",camera_matrix)
            image_with_axes = cv2.drawFrameAxes(color_image.copy(), camera_matrix, dist_coeffs, rvec, tvec, axis_length)

        cv2.imshow('RealSense', image_with_axes)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()