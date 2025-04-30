import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import math
import serial
import time

#for motor controller serial usb
ser = serial.Serial("/dev/ttyUSB0",9600) 

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable both color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

# Get camera intrinsics (focal length, principal point)
depth_sensor = profile.get_device().first_depth_sensor()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

# Load YOLO model
model = YOLO("/home/airlab/realsense/runs/detect/train/weights/best.pt")

classNames = ["Dandelion", "Lettuce"]

#output_file = 'output.mp4'
#fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
#out = cv2.VideoWriter(output_file, fourcc, 30.0, (1280, 480))

def get_3d_distance(depth_frame, x, y):
    depth = depth_frame.get_distance(x, y)
    if depth == 0:
        return None
    X = (x - intrinsics.ppx) * depth / intrinsics.fx
    Y = (y - intrinsics.ppy) * depth / intrinsics.fy
    Z = depth
    distance = math.sqrt(X**2 + Y**2 + Z**2)
    return distance

try:
    while True:
        # Get frames from RealSense camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue
        
        # Convert frames to OpenCV format
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Colorize depth frame for display
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Run YOLO on color image
        results = model(color_image, stream=True)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (255, 0, 255), 2)

                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                if 0 <= cls < len(classNames):
                    class_name = classNames[cls]
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    distance = depth_frame.get_distance(center_x, center_y)
                    label = f"{class_name} {confidence:.2f} ({distance:.2f}m)" if distance is not None else f"{class_name} {confidence:.2f} (No Depth)"
                    cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                    # give robot commands to go to the weed
                    #define the center of the robot and compare to where the object is located in the frame
                    #then give this a turning rate and set time.
                    # Use raw depth from frame (in meters)
                    depth = depth_frame.get_distance(center_x, center_y)

                    if depth is not None and depth > 0.1:  # Ignore very small or zero values
                        robot_center = 321  # image center
                        dx = center_x - robot_center
                        turn_angle = (dx / depth) / 10  # crude ratio — refine as needed

                        ser.write([64 + int(turn_angle)])
                        ser.write([192 - int(turn_angle)])
                        #time.sleep(1)

                        # Move forward based on depth
                        time_moving = depth / 0.25  # assuming 0.25 m/s speed
                        ser.write([74])  # forward_left
                        ser.write([202]) # forward_right
                        #time.sleep(time_moving)

                        # Stop motors
                        ser.write([64])  # stop_left
                        ser.write([192]) # stop_right
                    else:
                        print(f"Invalid or too-close depth at ({center_x}, {center_y}) = {depth:.2f}m")


        # Combine color and depth images
        combined_image = np.hstack((color_image, depth_colormap))

        # Write to video file
        #out.write(combined_image)

        # Display output
        cv2.imshow('RealSense YOLO (Color + Depth)', combined_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and release video writer
    pipeline.stop()
    #out.release()
    cv2.destroyAllWindows()
