# arducam_color_sampler.py
import cv2
import numpy as np

drawing = False
ix, iy = -1, -1
fx, fy = -1, -1
frame_copy = None

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, fx, fy, drawing, frame_copy
    if frame_copy is None:
        return

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
    x_start, x_end = min(x1, x2), max(x1, x2)
    y_start, y_end = min(y1, y2), max(y1, y2)

    roi = frame[y_start:y_end, x_start:x_end]
    if roi.size == 0:
        return

    avg_color = np.average(np.average(roi, axis=0), axis=0)
    avg_b, avg_g, avg_r = avg_color

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    min_hsv  = np.min(hsv_roi,  axis=(0, 1))
    max_hsv  = np.max(hsv_roi,  axis=(0, 1))
    avg_hsv  = np.average(np.average(hsv_roi, axis=0), axis=0)

    print("\n--- ROI Analysis ---")
    print(f"Average RGB: ({int(avg_r)}, {int(avg_g)}, {int(avg_b)})")
    print(f"Average HSV: ({int(avg_hsv[0])}, {int(avg_hsv[1])}, {int(avg_hsv[2])})")
    print(f"Suggested LOWER_BLUE = np.array([{min_hsv[0]}, {min_hsv[1]}, {min_hsv[2]}])")
    print(f"Suggested UPPER_BLUE = np.array([{max_hsv[0]}, {max_hsv[1]}, {max_hsv[2]}])")
    print("--------------------\n")

def main():
    global frame_copy, drawing, ix, iy, fx, fy

    cap = cv2.VideoCapture(1)   # change index if needed
    if not cap.isOpened():
        print("ERROR: Could not open Arducam. Check device index.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow('Arducam Sampler')
    cv2.setMouseCallback('Arducam Sampler', draw_rectangle)
    print("Click and drag to sample a colour region. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed.")
            break

        frame_copy = frame.copy()

        display = frame.copy()
        if drawing:
            cv2.rectangle(display, (ix, iy), (fx, fy), (0, 255, 0), 2)

        cv2.imshow('Arducam Sampler', display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()