import cv2
import numpy as np

# ================Physical Configs===================
# These should match the physical reference object you are clicking on
CARDBOARD_WIDTH_M  = 0.225
CARDBOARD_HEIGHT_M = 0.175
DIST_TO_CARDBOARD_M = 0.72

# BEV settings
PIXELS_PER_METER = 400
BEV_WIDTH  = 650
BEV_HEIGHT = 500
ROBOT_POS_X = BEV_WIDTH // 2   # adjust + / - to shift robot position left/right
ROBOT_POS_Y = BEV_HEIGHT        # adjust + / - to shift robot position up/down

# USB device index for the Arducam — check with: v4l2-ctl --list-devices
ARDUCAM_DEVICE_INDEX = 2

# Output file
SAVE_FILE = "arducam_calibration_data.npz"

# =========Mouse Input===========
calibration_points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(calibration_points) < 4:
            calibration_points.append((x, y))
            print(f"Point: {x}, {y}")

def order_points(pts):
    pts = np.array(pts, dtype="float32")
    y_sorted = pts[np.argsort(pts[:, 1])]
    top    = y_sorted[:2]
    bottom = y_sorted[2:]
    tl, tr = top[np.argsort(top[:, 0])]
    bl, br = bottom[np.argsort(bottom[:, 0])]
    return np.array([tl, tr, bl, br], dtype="float32")

def main():
    cap = cv2.VideoCapture(ARDUCAM_DEVICE_INDEX)
    if not cap.isOpened():
        print(f"ERROR: Cannot open Arducam at device index {ARDUCAM_DEVICE_INDEX}")
        print("Check available devices with: v4l2-ctl --list-devices")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", mouse_callback)

    print("===ARDUCAM CALIBRATION===")
    print("1) Click the 4 corners of the reference object.")
    print("2) Press 's' to SAVE and QUIT.")
    print("3) Press 'r' to RESET points.")
    print("4) Press ESC to quit without saving.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame capture failed.")
                break

            display = frame.copy()

            # Draw clicked points
            for pt in calibration_points:
                cv2.circle(display, pt, 5, (0, 0, 255), -1)

            if len(calibration_points) == 4:
                ordered_pts = order_points(calibration_points)
                cv2.polylines(display, [np.int32(ordered_pts)], True, (0, 255, 0), 2)
                cv2.putText(display, "Press 's' to Save", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Calibration", display)
            key = cv2.waitKey(1)

            if key == 27:  # ESC
                print("Exiting without saving.")
                break

            elif key == ord('r'):
                calibration_points.clear()
                print("Points reset.")

            elif key == ord('s'):
                if len(calibration_points) == 4:
                    print("Calculating matrix...")

                    src_pts = order_points(calibration_points)

                    # Destination points — same calculation as RealSense calibration
                    dst_bottom_y  = ROBOT_POS_Y - (DIST_TO_CARDBOARD_M * PIXELS_PER_METER)
                    dst_top_y     = dst_bottom_y - (CARDBOARD_HEIGHT_M  * PIXELS_PER_METER)
                    half_width_px = (CARDBOARD_WIDTH_M * PIXELS_PER_METER) / 2

                    dst_left_x  = ROBOT_POS_X - half_width_px
                    dst_right_x = ROBOT_POS_X + half_width_px

                    dst_pts = np.float32([
                        [dst_left_x,  dst_top_y   ],
                        [dst_right_x, dst_top_y   ],
                        [dst_left_x,  dst_bottom_y],
                        [dst_right_x, dst_bottom_y],
                    ])

                    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

                    np.savez(
                        SAVE_FILE,
                        matrix=matrix,
                        pixels_per_meter=PIXELS_PER_METER,
                        bev_width=BEV_WIDTH,
                        bev_height=BEV_HEIGHT,
                        robot_x=ROBOT_POS_X,
                        robot_y=ROBOT_POS_Y,
                    )

                    print(f"Calibration saved to '{SAVE_FILE}'")
                    print(f"  pixels_per_meter = {PIXELS_PER_METER}")
                    print(f"  bev_width        = {BEV_WIDTH}")
                    print(f"  bev_height       = {BEV_HEIGHT}")
                    print(f"  robot_x          = {ROBOT_POS_X}")
                    print(f"  robot_y          = {ROBOT_POS_Y}")
                    break
                else:
                    print("Requires 4 points to save.")

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()