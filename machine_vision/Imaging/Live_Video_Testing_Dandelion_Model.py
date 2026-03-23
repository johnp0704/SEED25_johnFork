import pyrealsense2 as rs
import numpy as np
import cv2
import math
from ultralytics import YOLO

#============Config===================
#path to trained model
model_path = r"C:\UVM\SEED\SEED25\Machine_Learning\YOLO\runs\dandelion_train_v1\weights\best.pt"

#==========Initialize Model=================
print(f"Loading YOLO model from: {model_path}")
model = YOLO(model_path)
print("Model loaded successfully.")

#class names
classNames = ["No Dandelion", "Dandelion"] 

#=============Initialize Realsense============
pipeline = rs.pipeline()
config = rs.config()

#enable streams
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

print("Starting RealSense Stream")
pipeline.start(config)
print("Stream started. Press ESC to exit.")

try:
    while True:
        #1) Wait for frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        #2) Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        #3) Run YOLO Inference directly on the current frame
        results = model(color_image, stream=True, verbose=False)

        #4) Process Detections and Draw Bounding Boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                #bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                #confidence
                conf = math.ceil((box.conf[0] * 100)) / 100

                #class name
                cls = int(box.cls[0])
                current_class = model.names[cls]

                #only draw if it detects a Dandelion
                if conf > 0.8: #threshold to filter weak detections
                    #draw rectangle
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

                    #draw label
                    label = f'{current_class} {conf}'
                    t_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                    c2 = x1 + t_size[0], y1 - t_size[1] - 3
                    cv2.rectangle(color_image, (x1, y1), c2, (0, 255, 0), -1, cv2.LINE_AA)  #filled box for text
                    cv2.putText(color_image, label, (x1, y1 - 2), 0, 1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)

        #5) Prepare Depth for Display (Colorize it)
        depth_colormap = cv2.convertScaleAbs(depth_image, alpha=0.03)
        depth_colormap_dim = cv2.cvtColor(depth_colormap, cv2.COLOR_GRAY2BGR)

        #6) Stack images horizontally (RGB with boxes and Depth)
        images = np.hstack((color_image, depth_colormap_dim))

        #7) Show live feed
        cv2.imshow('YOLO + RealSense', images)

        #exit condition
        key = cv2.waitKey(1)
        if key == 27:  # esc
            break

finally:
    #stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Program Terminated.")