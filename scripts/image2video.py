import os
import subprocess

# Base directory where the folders are located
base_dir = "/home/yif/Documents/KTH/dynamicCage/evaluation/real-world"
tasks = [
    "circle-pushes-rectangle", "jaw-pushes-rectangle",
    "circle-pushes-triangle", "jaw-pushes-triangle",
    "circle-pushes-concave", "jaw-pushes-concave",
    "circle-pushes-convex", "jaw-pushes-convex",
    "circle-pushes-irregular", "jaw-pushes-irregular"
]

# Target directory for the videos
video_dir = os.path.join(base_dir, "videos")
os.makedirs(video_dir, exist_ok=True)

# Iterate through each task
for task in tasks:
    task_dir = os.path.join(base_dir, task)
    # Iterate through each subfolder in the task directory
    for id in range(12):  # Assuming subfolders are named "0" to "11"
        subfolder_path = os.path.join(task_dir, str(id), "captured_images")
        # Construct the ffmpeg command
        output_video_path = os.path.join(video_dir, f"vid_{task}_{id}.mp4")
        ffmpeg_command = [
            "ffmpeg", "-framerate", "33", "-start_number", "50",
            "-i", os.path.join(subfolder_path, "color_image_%d.png"),
            "-vframes", "100", "-c:v", "libx264", "-pix_fmt", "yuv420p",
            output_video_path
        ]
        # Execute the ffmpeg command
        try:
            subprocess.run(ffmpeg_command, check=True)
            print(f"Video created: {output_video_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error creating video for {task} {id}: {e}")

print("Video generation completed.")