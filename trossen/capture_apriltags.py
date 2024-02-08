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
import csv
import os
import pandas as pd
import time

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


# Camera intrinsic parameters
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
tag_size = 0.032  # Adjust based on your actual tag size, TODO: measure this
axis_length = 0.03  # For vis (meters)
tag_world_id = 0
tag_objects_id = [1,2,]

# Initialize the detector
detector = apriltag.Detector()

###########################################
# Capture images
###########################################

# Initialize and start the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Directory to save images
image_dir = "captured_images"
os.makedirs(image_dir, exist_ok=True)

# Number of frames to capture
N = 100
data = []
for i in range(N):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue
    
    # Save timestamp and image
    timestamp = color_frame.get_timestamp()  # Timestamp in milliseconds
    img_filename = f"color_image_{i}.png"
    cv2.imwrite(os.path.join(image_dir, img_filename), np.asanyarray(color_frame.get_data()))
    data.append([timestamp, img_filename])
    print(f"Saved {img_filename} with timestamp {timestamp} ms")

pipeline.stop()

# CSV file for timestamps
csv_file = "timestamps.csv"
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp_ms", "filename"])
    writer.writerows(data)

###########################################
# Detect AprilTags
###########################################

# Load timestamps and filenames
timestamps_df = pd.read_csv(csv_file)

# Prepare data for CSV saving
visualize = True
tag_corners_3d = np.array([[-tag_size / 2, -tag_size / 2, 0],
                            [ tag_size / 2, -tag_size / 2, 0],
                            [ tag_size / 2,  tag_size / 2, 0],
                            [-tag_size / 2,  tag_size / 2, 0]])

# Process each saved image
rvec_world_rgb, tvec_world_rgb, valid_count = None, None, 0
first_timestamp = None # in sec
pose_data = []
for _, row in timestamps_df.iterrows():
    img_path = os.path.join(image_dir, row['filename'])
    color_image = cv2.imread(img_path)
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    # Perform detection
    detections = detector.detect(gray)
    
    # Your pose calculation and transformation logic here
    transformation_data = []
    for detection in detections:
        if detection.tag_id not in tag_objects_id+[tag_world_id,]:
            continue
        tag_corners_2d = np.array(detection.corners, dtype=np.float32)
        
        # Use solvePnP to get the tag pose
        retval, rvec, tvec = cv2.solvePnP(tag_corners_3d, tag_corners_2d, camera_matrix, dist_coeffs)
        if first_timestamp is None:
            first_timestamp = row['timestamp_ms']
        curr_time = (row['timestamp_ms'] - first_timestamp)
        transformation_data.append((curr_time, detection.tag_id, rvec, tvec))

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
    if len(transformation_data) < 3:
        continue

    # Calculate the transformation to the world frame for tags with other ids
    rt_temp = {'timestamp': None, 'object0': None, 'object1': None}
    for curr_time, tag_id, rvec, tvec in transformation_data:
        rt_temp["timestamp"] = curr_time
        if tag_id == tag_world_id:
            # Get the transformation from the world to the rgb camera frame
            rvec_rgb_world, tvec_rgb_world = rvec, tvec
            rvec_world_rgb, tvec_world_rgb = inverse_transformation(rvec_rgb_world, tvec_rgb_world)

            # Average the transformation over time TODO

        if tag_id in tag_objects_id:
            # Get the transformation from the world to the object frames
            res = cv2.composeRT(rvec, tvec, rvec_world_rgb, tvec_world_rgb,)
            rvec_world_tag, tvec_world_tag = res[0], res[1]
            if tag_id == tag_objects_id[0]:
                rt_temp["object0"] = (tag_id, rvec_world_tag, tvec_world_tag)
            if tag_id == tag_objects_id[1]:
                rt_temp["object1"] = (tag_id, rvec_world_tag, tvec_world_tag)
            # print(f"Tag {tag_id} pose in world frame (rvec, tvec): {rvec_world_tag}, {tvec_world_tag}")
            # rt_temp.append([tag_id, rvec_world_tag, tvec_world_tag])
    pose_data.append(rt_temp)
    time.sleep(0.1)

###########################################
# Calculate velocity
###########################################

# Velocity computation
data_to_save = []
for i in range(1, len(pose_data)):
    print(i)
    curr_data = pose_data[i]
    prev_data = pose_data[i-1]
    curr_time = curr_data["timestamp"]
    prev_time = prev_data["timestamp"]
    if curr_time == prev_time:
        continue
    curr_time_sec = curr_time / 1000.0
    prev_time_sec = prev_time / 1000.0
    row = [curr_time_sec,]
    for j in ["object0", "object1"]:
        curr_tag_id, curr_rvec, curr_tvec = curr_data[j]
        prev_tag_id, prev_rvec, prev_tvec = prev_data[j]
        vel_tvec = (curr_tvec - prev_tvec) / (curr_time_sec - prev_time_sec)
        vel_rvec = (curr_rvec - prev_rvec) / (curr_time_sec - prev_time_sec)
        row.extend([curr_tag_id, curr_tvec[0][0], curr_tvec[1][0], curr_rvec[2][0], vel_tvec[0][0], vel_tvec[1][0], vel_rvec[2][0]])
    data_to_save.append(row)

# Save the results to a CSV file
results_csv = "apriltag_results.csv"
with open(results_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp_sec", 
                     "tag_id1", "tvec_world_tag_1_x", "tvec_world_tag_1_y", "rvec_world_tag_1_z",
                     "vel_tvec_world_tag_1_x", "vel_tvec_world_tag_1_y", "vel_rvec_world_tag_1_z",
                     "tag_id2", "tvec_world_tag_2_x", "tvec_world_tag_2_y", "rvec_world_tag_2_z",
                     "vel_tvec_world_tag_2_x", "vel_tvec_world_tag_2_y", "vel_rvec_world_tag_2_z"])
    for data in data_to_save:
        writer.writerow(data)

print(f"Detection results saved to {results_csv}")