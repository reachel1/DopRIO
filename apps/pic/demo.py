ffmpeg -framerate 24 -i frame/frame_%d.png -c:v libx264 -pix_fmt yuv420p xx.mp4
ffmpeg -i xx.mp4 xx.gif
