import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import imutils
import math
print("Environment Ready")

# Setup:
pipe = rs.pipeline()
cfg = rs.config()
#cfg.enable_device_from_file("../object_detection.bag")
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(cfg)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(5):
  pipe.wait_for_frames()
  
# Store next frameset for later processing:
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()

# Cleanup:
pipe.stop()
print("Frames Captured")

color = np.asanyarray(color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.imshow(color)

colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
plt.imshow(colorized_depth)

# Create alignment primitive with color as its target stream:
align = rs.align(rs.stream.color)
frameset = align.process(frameset)

# Update color and depth frames:
aligned_depth_frame = frameset.get_depth_frame()
colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

# Show the two frames together:
images = np.hstack((color, colorized_depth))
plt.imshow(images)

PI = 3.1415926

frameWidth = 640
frameHeight = 480

def update_perspective(val):
    alpha = (cv2.getTrackbarPos("Alpha", "Result") - 90) * PI / 180
    beta = (cv2.getTrackbarPos("Beta", "Result") - 90) * PI / 180
    gamma = (cv2.getTrackbarPos("Gamma", "Result") - 90) * PI / 180
    focalLength = cv2.getTrackbarPos("f", "Result")
    dist = cv2.getTrackbarPos("Distance", "Result")

    image_size = (frameWidth, frameHeight)
    w, h = image_size

    A1 = np.array([[1, 0, -w / 2],
                [0, 1, -h / 2],
                [0, 0, 0],
                [0, 0, 1]], dtype=np.float32)

    RX = np.array([[1, 0, 0, 0],
                [0, math.cos(alpha), -math.sin(alpha), 0],
                [0, math.sin(alpha), math.cos(alpha), 0],
                [0, 0, 0, 1]], dtype=np.float32)

    RY = np.array([[math.cos(beta), 0, -math.sin(beta), 0],
                [0, 1, 0, 0],
                [math.sin(beta), 0, math.cos(beta), 0],
                [0, 0, 0, 1]], dtype=np.float32)

    RZ = np.array([[math.cos(gamma), -math.sin(gamma), 0, 0],
                [math.sin(gamma), math.cos(gamma), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]], dtype=np.float32)

    R = np.dot(np.dot(RX, RY), RZ)

    T = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, dist],
                [0, 0, 0, 1]], dtype=np.float32)

    K = np.array([[focalLength, 0, w / 2, 0],
                [0, focalLength, h / 2, 0],
                [0, 0, 1, 0]], dtype=np.float32)

    transformationMat = np.dot(np.dot(np.dot(K, T), R), A1)

    destination = cv2.warpPerspective(source, transformationMat, image_size, flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)

    cv2.imshow("Result", destination)

source = color  # Replace with your image file path

cv2.namedWindow("Result", cv2.WINDOW_NORMAL)

cv2.createTrackbar("Alpha", "Result", 90, 180, update_perspective)
cv2.createTrackbar("Beta", "Result", 90, 180, update_perspective)
cv2.createTrackbar("Gamma", "Result", 90, 180, update_perspective)
cv2.createTrackbar("f", "Result", 500, 2000, update_perspective)
cv2.createTrackbar("Distance", "Result", 500, 2000, update_perspective)

update_perspective(0)
cv2.waitKey(0)
cv2.destroyAllWindows()
