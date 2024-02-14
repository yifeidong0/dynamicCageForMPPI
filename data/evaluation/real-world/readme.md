Make a video from the captured images:
ffmpeg -framerate 33 -start_number 50 -i color_image_%d.png -vframes 100 -c:v libx264 -pix_fmt yuv420p output_video.mp4