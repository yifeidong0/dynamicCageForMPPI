The command for compiling .ppm images into a video:

yif@yif-GS66:~/Documents/KTH/git/pyOptimalMotionPlanning$ ffmpeg -framerate 12 -i ./data/ao_rrt/image%04d.ppm -c:v libx264 -vf "fps=12,format=yuv420p" output.mp4