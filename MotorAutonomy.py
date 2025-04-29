import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import math
import serial
import time
import RPi.GPIO as GPIO

#time.sleep(20)


#for GPIO write to high or low
pin = 12
button_pin = 18

#pin setup
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
GPIO.setup(pin, GPIO.OUT)  #  pin set as output
GPIO.output(pin, GPIO.HIGH) #start pin as high (arm is up)
GPIO.setup(button_pin, GPIO.IN)  #  pin set as input
prev_value = 1


#for motor controller serial usb
ser = serial.Serial("/dev/ttyUSB0",9600) 
    #stop motors
ser.write([64])   # Stop left motor
ser.write([192])  # Stop right motor

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
model = YOLO("/home/airlab/realsense/runs/detect/train4/weights/best.pt")

classNames = ["3"]

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
        current_value = GPIO.input(button_pin)
        print("current value ", current_value)
        if(current_value != prev_value):
            time.sleep(1)
            break
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

                    if depth is not None and depth > 0.1:  # Ignore invalid or too-close depth
                        GPIO.output(pin, GPIO.HIGH)  # Raise auger while moving

                        robot_center = 321
                        d_left_right = center_x - robot_center  # Negative = left, Positive = right

                        # Base forward speed (low and safe)
                        base_left = 70
                        base_right = 198

                        # Apply proportional adjustment (limit max diff to prevent extreme turns)
                        Kp = 0.05
                        correction = Kp * d_left_right
                        print("correction before clamping: ", correction)
                        correction = max(min(correction, 5), -5)  # Clamp correction to +/-5
                        print("Correction: ", correction)

                        left_speed = int(base_left + correction)
                        right_speed = int(base_right - correction)

                       # Ensure motor values stay within bounds
                        left_speed = max(min(left_speed, 79), 64)
                        right_speed = max(min(right_speed, 207), 192)

                        # Send speeds to Sabertooth
                        ser.write([left_speed])
                        print("left speed: " , left_speed)
                        ser.write([right_speed])
                        print("right speed: " , right_speed)

                    else:
                        # Stop motors
                        ser.write([64])   # Stop left motor
                        ser.write([192])  # Stop right motor
                        print(f"Invalid or too-close depth at ({center_x}, {center_y}) = {depth:.2f}m")
                        time.sleep(5)
                        GPIO.output(pin, GPIO.LOW)
                                       
                    # Drop auger after reaching target
                    time.sleep(5)
                    GPIO.output(pin, GPIO.LOW)
        
         #stop motors
        ser.write([64])   # Stop left motor
        ser.write([192])  # Stop right motor


finally:
    #stop motors
    ser.write([64])   # Stop left motor
    ser.write([192])  # Stop right motor
    # Stop the pipeline and release video writer
    pipeline.stop()
    #out.release()
    cv2.destroyAllWindows()
    GPIO.output(pin, GPIO.HIGH)   # explicitly turn off before cleanup
    GPIO.cleanup()
