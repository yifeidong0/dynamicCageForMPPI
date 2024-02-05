"""
frame_id: "camera_color_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [614.9035034179688, 0.0, 323.47271728515625, 0.0, 614.9575805664062, 237.75799560546875, 0.0, 0.0, 1.0]
(fx = 614.9035034179688, fy = 614.9575805664062, cx = 323.47271728515625, and cy = 237.75799560546875)
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [614.9035034179688, 0.0, 323.47271728515625, 0.0, 0.0, 614.9575805664062, 237.75799560546875, 0.0, 0.0, 0.0, 1.0, 0.0]
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag

def inverse_transformation(rvec, tvec,):
    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)

    # Construct the 4x4 transformation matrix
    T = np.zeros((4, 4), dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    T[3, 3] = 1.0

    # Calculate the inverse transformation
    T_inv = np.linalg.inv(T)

    # Extract rotation matrix and translation vector from the inverse transformation
    R_inv = T_inv[:3, :3]
    tvec_inv = T_inv[:3, 3]

    # Convert inverse rotation matrix back to a rotation vector
    rvec_inv, _ = cv2.Rodrigues(R_inv)

    return rvec_inv, tvec_inv


# Initialize the detector
detector = apriltag.Detector()

# Camera intrinsic parameters
# Replace these values with your camera's specific parameters
intrinsics = {
    "fx": 614.9035034179688,  # Focal length in x
    "fy": 614.9575805664062,  # Focal length in y
    "cx": 323.47271728515625,  # Principal point x
    "cy": 237.75799560546875,  # Principal point y
}
camera_matrix = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                          [0, intrinsics["fy"], intrinsics["cy"]],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Tag size (meters)
tag_size = 0.025  # Adjust based on your actual tag size, TODO: measure this
axis_length = 0.01  # For vis (meters)
tag_world_id = 0

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
        
        timestamp = color_frame.get_timestamp()
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        transformation_data = []
        for detection in detections:
            # Assuming square tags, for simplicity
            tag_corners_2d = np.array(detection.corners, dtype=np.float32)
            tag_corners_3d = np.array([[-tag_size / 2, -tag_size / 2, 0],
                                       [ tag_size / 2, -tag_size / 2, 0],
                                       [ tag_size / 2,  tag_size / 2, 0],
                                       [-tag_size / 2,  tag_size / 2, 0]])

            # Use solvePnP to get the tag pose
            retval, rvec, tvec = cv2.solvePnP(tag_corners_3d, tag_corners_2d, camera_matrix, dist_coeffs)
            transformation_data.append((detection.tag_id, rvec, tvec))

            # Draw the detected tag
            for idx, point in enumerate(detection.corners):
                pt = tuple(point.astype(int))
                cv2.circle(color_image, pt, 5, (0, 255, 0), thickness=-1)
            cv2.putText(color_image, str(detection.tag_id), tuple(detection.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Draw the pose on the image
            image_with_axes = cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec, tvec, axis_length)

        cv2.imshow('RealSense', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        # Calculate the transformation to the world frame for tags with other ids
        rvec_world_rgb, tvec_world_rgb = None, None
        for tag_id, rvec, tvec in transformation_data:
            if tag_id == tag_world_id:
                # Get the transformation from the tag to the world frame
                rvec_rgb_world, tvec_rgb_world = rvec, tvec
                rvec_world_rgb, tvec_world_rgb = inverse_transformation(rvec_rgb_world, tvec_rgb_world)
            if tag_id != tag_world_id:
                res = cv2.composeRT(rvec, tvec, rvec_world_rgb, tvec_world_rgb,)
                rvec_world_tag, tvec_world_tag = res[0], res[1]
                print(f"Tag {tag_id} pose in world frame (rvec, tvec): {rvec_world_tag}, {tvec_world_tag}")
finally:
    pipeline.stop()
