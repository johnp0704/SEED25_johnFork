import numpy as np
import cv2
import math
from ultralytics import YOLO

#============Config===================
# Path to trained model
model_path = r"C:\UVM\SEED\SEED25\Machine_Learning\YOLOred\runs\red_train_7\weights\best.pt"

# Camera ID (0 is usually the default built-in webcam. Change to 1 or 2 if using an external USB camera alongside a built-in one)
camera_id = 3

#==========Initialize Model=================
print(f"Loading YOLO model from: {model_path}")
model = YOLO(model_path)
print("Model loaded successfully.")

# Class names
classNames = ["No Red", "Red"] 

#=============Initialize Arducam============
cap = cv2.VideoCapture(camera_id)

# Set resolution (Matching your previous 1280x720 setup)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print(f"Error: Could not open camera with ID {camera_id}.")
    exit()

print("Starting Arducam Stream")
print("Stream started. Press ESC to exit.")

try:
    while True:
        # 1) Read frame from USB camera
        success, color_image = cap.read()

        if not success:
            print("Failed to grab frame.")
            break

        # 2) Run YOLO Inference directly on the current frame
        results = model(color_image, stream=True, verbose=False)

        # 3) Process Detections and Draw Bounding Boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Confidence
                conf = math.ceil((box.conf[0] * 100)) / 100

                # Class name
                cls = int(box.cls[0])
                current_class = model.names[cls]

                # Only draw if confidence is above threshold
                if conf > 0.8: # Threshold to filter weak detections
                    # Draw rectangle
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

                    # Draw label
                    label = f'{current_class} {conf}'
                    t_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                    c2 = x1 + t_size[0], y1 - t_size[1] - 3
                    cv2.rectangle(color_image, (x1, y1), c2, (0, 255, 0), -1, cv2.LINE_AA)  # Filled box for text
                    cv2.putText(color_image, label, (x1, y1 - 2), 0, 1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)

        # 4) Show live feed
        cv2.imshow('YOLO + Arducam', color_image)

        # Exit condition
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break

finally:
    # Stop streaming and clean up
    cap.release()
    cv2.destroyAllWindows()
    print("Program Terminated.")