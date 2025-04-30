import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import imutils
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


# Standard OpenCV boilerplate for running the net:
height, width = color.shape[:2]
expected = 300
aspect = width / height
resized_image = cv2.resize(color, (round(expected * aspect), expected))
crop_start = round(expected * (aspect - 1) / 2)
crop_img = resized_image[0:expected, crop_start:crop_start+expected]


proto_file = "/home/ron/MobileNetSSD/MobileNetSSD_deploy.prototxt.txt"
model_file = "/home/ron/MobileNetSSD/MobileNetSSD_deploy.caffemodel"

net = cv2.dnn.readNetFromCaffe(proto_file, model_file)
inScaleFactor = 0.007843
meanVal       = 127.53
classNames = ("background", "aeroplane", "bicycle", "bird", "boat",
              "bottle", "bus", "car", "cat", "chair",
              "cow", "diningtable", "dog", "horse",
              "motorbike", "person", "pottedplant",
              "sheep", "sofa", "train", "tvmonitor")

blob = cv2.dnn.blobFromImage(crop_img, inScaleFactor, (expected, expected), meanVal, False)
net.setInput(blob, "data")
detections = net.forward("detection_out")

label = detections[0,0,0,1]
conf  = detections[0,0,0,2]
xmin  = detections[0,0,0,3]
ymin  = detections[0,0,0,4]
xmax  = detections[0,0,0,5]
ymax  = detections[0,0,0,6]

className = classNames[int(label)]

cv2.rectangle(crop_img, (int(xmin * expected), int(ymin * expected)), 
             (int(xmax * expected), int(ymax * expected)), (255, 255, 255), 2)
cv2.putText(crop_img, className, 
            (int(xmin * expected), int(ymin * expected) - 5),
            cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255))

plt.imshow(crop_img)

scale = height / expected
xmin_depth = int((xmin * expected + crop_start) * scale)
ymin_depth = int((ymin * expected) * scale)
xmax_depth = int((xmax * expected + crop_start) * scale)
ymax_depth = int((ymax * expected) * scale)
xmin_depth,ymin_depth,xmax_depth,ymax_depth
cv2.rectangle(colorized_depth, (xmin_depth, ymin_depth), 
             (xmax_depth, ymax_depth), (255, 255, 255), 2)
plt.imshow(colorized_depth)

depth = np.asanyarray(aligned_depth_frame.get_data())
# Crop depth data:
depth = depth[xmin_depth:xmax_depth,ymin_depth:ymax_depth].astype(float)

# Get data scale from the device and convert to meters
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
depth = depth * depth_scale
dist,_,_,_ = cv2.mean(depth)
print("Detected a {0} {1:.3} meters away.".format(className, dist))

#####circles = 

# Compute the center of the bounding box
center_x = (xmin_depth + xmax_depth) / 2
center_y = (ymin_depth + ymax_depth) / 2

# Represent the object as a horizontal line (its lateral extent)
line_start = (xmin_depth, center_y)
line_end = (xmax_depth, center_y)

# Assume an average depth (distance along the robot’s forward direction)
dist = dist  # example value in meters
print("Detected object {:.3f} meters away.".format(dist))

# --- Define Conversion Factor ---
# Assume 30 pixels correspond to 1 meter at this depth.
pixels_per_meter = 200

# --- Convert Object Coordinates to Meters ---
# The robot’s center in the image (in pixels) is assumed to be at x = 320.
robot_pixel_x = 320
# Convert the object’s x coordinates from pixels to meters relative to the robot’s center:
object_xmin_m = (xmin_depth - robot_pixel_x) / pixels_per_meter
object_xmax_m = (xmax_depth - robot_pixel_x) / pixels_per_meter
object_center_x_m = (object_xmin_m + object_xmax_m) / 2

# Use the measured depth as the object’s y position in meters.
object_y_m = dist

# Define the object’s lateral extent as a horizontal line in the top‐down view:
line_start = (object_xmin_m, object_y_m)
line_end = (object_xmax_m, object_y_m)

# --- Define Robot Position in Meters ---
# Set the robot’s position as the origin (0, 0)
robot_x_m = 0
robot_y_m = 0

# --- Define Turning Arcs in Meters ---
# For a pleasing avoidance path, we choose an arc radius proportional to the distance.
# Previously, arc_radius (in pixels) was computed as int(dist * 30), so in meters:
arc_radius = dist - 0.25  # (since arc_radius_m = (dist*30)/30 = dist)
print("Arc radius (meters):", arc_radius)

# Left-turn arc:
# Instantaneous center is to the left at (-arc_radius, 0)
center_left = (-arc_radius, 0)
theta_left = np.linspace(0, np.pi/2, 100)
# Parameterize: starting at (0,0) when theta=0, the arc moves upward.
x_left = center_left[0] + arc_radius * np.cos(theta_left)
y_left = center_left[1] + arc_radius * np.sin(theta_left)

# Right-turn arc:
# Instantaneous center is to the right at (arc_radius, 0)
center_right = (arc_radius, 0)
theta_right = np.linspace(np.pi, np.pi/2, 100)
x_right = center_right[0] + arc_radius * np.cos(theta_right)
y_right = center_right[1] + arc_radius * np.sin(theta_right)

# --- Plotting in Meters ---
fig, ax = plt.subplots(figsize=(8, 8))

# Plot the object’s lateral extent (in meters)
ax.plot([line_start[0], line_end[0]], [line_start[1], line_end[1]], 
        color='blue', linewidth=4, label='Object Dimension')

# Mark the object’s center
ax.plot(object_center_x_m, object_y_m, 'ro', label='Object Center')

# Plot the left and right avoidance arcs
if (object_center_x_m >= 0):
  ax.plot(x_left, y_left, 'g--', linewidth=3, label='Left Avoidance Arc')

if (object_center_x_m < 0):
  ax.plot(x_right, y_right, 'm--', linewidth=3, label='Right Avoidance Arc')

# Mark the robot’s position (origin)
ax.plot(robot_x_m, robot_y_m, 'ko', markersize=8, label='Robot Position')

# Annotate the detected distance near the object
ax.text(object_center_x_m, object_y_m - 0.1, f"{dist:.3f} m away", 
        color='black', fontsize=12, ha='center')

# Set plot limits (in meters)
ax.set_xlim(-dist * 1.5, dist * 1.5)
ax.set_ylim(0, dist * 2)

plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Top-down View with Avoidance Arcs (Meters)")
plt.legend()
plt.grid(True)
plt.show()
