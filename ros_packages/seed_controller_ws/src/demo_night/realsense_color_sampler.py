import pyrealsense2 as rs
import numpy as np
import cv2

# Global variables for mouse callback
drawing = False
ix, iy = -1, -1
fx, fy = -1, -1
frame_copy = None

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, fx, fy, drawing, frame_copy
    
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        fx, fy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            fx, fy = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        fx, fy = x, y
        analyze_roi(ix, iy, fx, fy, frame_copy)

def analyze_roi(x1, y1, x2, y2, frame):
    # Ensure coordinates are top-left to bottom-right
    x_start, x_end = min(x1, x2), max(x1, x2)
    y_start, y_end = min(y1, y2), max(y1, y2)
    
    roi = frame[y_start:y_end, x_start:x_end]
    if roi.size == 0:
        return

    # Calculate average RGB
    avg_color_per_row = np.average(roi, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    avg_b, avg_g, avg_r = avg_color
    
    # Convert ROI to HSV and calculate min/max/avg for masking limits
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    min_hsv = np.min(hsv_roi, axis=(0, 1))
    max_hsv = np.max(hsv_roi, axis=(0, 1))
    avg_hsv = np.average(np.average(hsv_roi, axis=0), axis=0)

    print("\n--- ROI Analysis ---")
    print(f"Average RGB: ({int(avg_r)}, {int(avg_g)}, {int(avg_b)})")
    print(f"Average HSV: ({int(avg_hsv[0])}, {int(avg_hsv[1])}, {int(avg_hsv[2])})")
    print(f"Suggested HSV Mask Lower: {min_hsv}")
    print(f"Suggested HSV Mask Upper: {max_hsv}")
    print("--------------------\n")

def main():
    global frame_copy
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    cv2.namedWindow('RealSense Sampler')
    cv2.setMouseCallback('RealSense Sampler', draw_rectangle)

    print("Click and drag to select an area for color sampling. Press 'q' to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            frame_copy = frame.copy()

            if drawing:
                cv2.rectangle(frame, (ix, iy), (fx, fy), (0, 255, 0), 2)

            cv2.imshow('RealSense Sampler', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()