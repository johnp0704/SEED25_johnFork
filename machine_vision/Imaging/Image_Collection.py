import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime
import time

# Define absolute paths for saving images (Change if needed)
rgb_folder = r"C:\UVM\SEED\Images\Preliminary Images\RGB"
depth_folder = r"C:\UVM\SEED\Images\Preliminary Images\Depth"

# Create folders if they don't exist
os.makedirs(rgb_folder, exist_ok=True)
os.makedirs(depth_folder, exist_ok=True)

def get_next_index(folder, prefix):
    # List files in folder starting with prefix
    files = [f for f in os.listdir(folder) if f.startswith(prefix) and f.endswith(".png")]
    
    indices = []
    for f in files:
        try:
            # Filename format: prefix + YYYYMMDD + _ + index + .png
            # Split by '_' and take last part before .png as index
            index_part = f.split('_')[-1].split('.')[0]
            indices.append(int(index_part))
        except:
            continue
    
    if indices:
        return max(indices) + 1
    else:
        return 1


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

print("SPACE to capture a photo. ESC to exit.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.convertScaleAbs(depth_image, alpha=0.03)

        images = np.hstack((color_image, cv2.cvtColor(depth_colormap, cv2.COLOR_GRAY2BGR)))
        cv2.imshow('RealSense RGB and Depth Stream', images)

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break
        elif key == 32:  # SPACE

            date_str = datetime.now().strftime("%Y%m%d")
            rgb_index = get_next_index(rgb_folder, "photo_")
            depth_index = get_next_index(depth_folder, "depth_")  # or "photo_" if you want to keep same indexing for both

            rgb_filename = os.path.join(rgb_folder, f"photo_{date_str}_{rgb_index}.png")
            depth_filename = os.path.join(depth_folder, f"depth_{date_str}_{depth_index}.png")

            cv2.imwrite(rgb_filename, color_image)
            cv2.imwrite(depth_filename, depth_image)

            print(f"Saved RGB image to: {rgb_filename}")
            print(f"Saved Depth image to: {depth_filename}")

            # Optional small delay to avoid too rapid saving
            time.sleep(0.1)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
