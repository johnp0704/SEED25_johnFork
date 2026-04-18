import pyrealsense2 as rs
import numpy as np
import cv2
import math
from ultralytics import YOLO

# ============ Config ===================
model_path = r"C:\UVM\SEED\SEED25\Machine_Learning\YOLOred_DEMO\runs\red_train_2\weights\best.pt"
camera_id = 3
CONFIDENCE_THRESHOLD = 0.2

# ========== Initialize Model =================
print(f"Loading YOLO model from: {model_path}")
model = YOLO(model_path)
print("Model loaded successfully.")

# ============ Helper Function =================
def process_and_draw(image, model):
    """Runs YOLO inference and draws bounding boxes on the provided image."""
    results = model(image, stream=True, verbose=False)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Confidence and Class
            conf = math.ceil((box.conf[0] * 100)) / 100
            cls = int(box.cls[0])
            current_class = model.names[cls]

            # Only draw if confidence is above threshold
            if conf > CONFIDENCE_THRESHOLD:
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                label = f'{current_class} {conf}'
                t_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                c2 = x1 + t_size[0], y1 - t_size[1] - 3
                cv2.rectangle(image, (x1, y1), c2, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.putText(image, label, (x1, y1 - 2), 0, 1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)
                
    return image

# ============= Initialize Cameras ============
# 1. RealSense Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
print("Starting RealSense Stream...")
pipeline.start(config)

# 2. Arducam Setup
cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print(f"Error: Could not open camera with ID {camera_id}.")
    exit()
print("Starting Arducam Stream...")

print("Streams started. Press ESC to exit.")

try:
    while True:
        # 1) Grab RealSense frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # 2) Grab Arducam frame
        success, arducam_image = cap.read()

        if not color_frame or not depth_frame or not success:
            continue

        # Convert RealSense frames to numpy arrays
        rs_color_image = np.asanyarray(color_frame.get_data())
        rs_depth_image = np.asanyarray(depth_frame.get_data())
        
        # Safety Check: Ensure Arducam image matches RealSense dimensions exactly before stacking
        if arducam_image.shape != rs_color_image.shape:
            arducam_image = cv2.resize(arducam_image, (rs_color_image.shape[1], rs_color_image.shape[0]))

        # 3) Run Inference & Draw Boxes
        rs_color_image = process_and_draw(rs_color_image, model)
        arducam_image = process_and_draw(arducam_image, model)

        # 4) Stack RGB Images Vertically (RealSense Top, Arducam Bottom)
        stacked_rgb = np.vstack((rs_color_image, arducam_image))

        # 5) Process RealSense Depth
        depth_colormap = cv2.convertScaleAbs(rs_depth_image, alpha=0.03)
        depth_colormap_dim = cv2.cvtColor(depth_colormap, cv2.COLOR_GRAY2BGR)

        # 6) Display Windows
        # Note: Stacking two 720p images creates a 1440p tall window. 
        # If this extends off the bottom of your screen, uncomment the resize line below.
        cv2.namedWindow('Stacked RGB: RealSense (Top) | Arducam (Bottom)', cv2.WINDOW_NORMAL)
        
        cv2.imshow('Stacked RGB: RealSense (Top) | Arducam (Bottom)', stacked_rgb) # Replace 'stacked_rgb' with 'display_stacked' if scaling down
        cv2.imshow('RealSense Depth', depth_colormap_dim)

        # Exit condition
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break

finally:
    # Clean up and release hardware
    pipeline.stop()
    cap.release()
    cv2.destroyAllWindows()
    print("Program Terminated.")