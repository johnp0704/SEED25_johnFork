import pyrealsense2 as rs
import numpy as np
import cv2
import os

# ================Physical Configs===================
CARDBOARD_WIDTH_M = 0.225
CARDBOARD_HEIGHT_M = 0.175
DIST_TO_CARDBOARD_M = 0.42

#BEV settings
PIXELS_PER_METER = 400
BEV_WIDTH = 650
BEV_HEIGHT = 500
ROBOT_POS_X = BEV_WIDTH // 2 + 50 #+ is tuning parameter
ROBOT_POS_Y = BEV_HEIGHT - 50 #- is tuning parameter

#output directory
SAVE_FILE = r"C:\UVM\SEED\SEED25\machine_vision\Video_Detection\calibration_data.npz"

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
    top = y_sorted[:2]
    bottom = y_sorted[2:]
    tl, tr = top[np.argsort(top[:, 0])]
    bl, br = bottom[np.argsort(bottom[:, 0])]
    return np.array([tl, tr, bl, br], dtype="float32")

def main():
    #initialize RealSense d435i
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", mouse_callback)

    print(f"===CALIBRATION===")
    print(f"1) Click the 4 corners of the reference.")
    print(f"2) Press 's' to SAVE and QUIT.")
    print(f"3) Press 'r' to RESET points.")
    print(f"4) Press 'ESC' to Quit without saving.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            frame = np.asanyarray(color_frame.get_data())
            display = frame.copy()

            #draw points
            for pt in calibration_points:
                cv2.circle(display, pt, 5, (0, 0, 255), -1)
            
            if len(calibration_points) == 4:
                #draw box
                ordered_pts = order_points(calibration_points)
                cv2.polylines(display, [np.int32(ordered_pts)], True, (0, 255, 0), 2)
                cv2.putText(display, "Press 's' to Save", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Calibration", display)
            key = cv2.waitKey(1)

            if key == 27: #ESC
                print("Exiting without saving.")
                break
            
            elif key == ord('r'): #reset
                calibration_points.clear()
                print("Points reset.")

            elif key == ord('s'): #save
                if len(calibration_points) == 4:
                    print("Calculating Matrix")
                    
                    #source points
                    src_pts = order_points(calibration_points)
                    
                    #destination points
                    dst_bottom_y = ROBOT_POS_Y - (DIST_TO_CARDBOARD_M * PIXELS_PER_METER)
                    dst_top_y = dst_bottom_y - (CARDBOARD_HEIGHT_M * PIXELS_PER_METER)
                    half_width_px = (CARDBOARD_WIDTH_M * PIXELS_PER_METER) / 2
                    
                    dst_left_x = ROBOT_POS_X - half_width_px
                    dst_right_x = ROBOT_POS_X + half_width_px
                    
                    dst_pts = np.float32([
                        [dst_left_x,  dst_top_y],
                        [dst_right_x, dst_top_y],
                        [dst_left_x,  dst_bottom_y],
                        [dst_right_x, dst_bottom_y]
                    ])

                    #generate matrix
                    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
                    
                    #save to file
                    np.savez(SAVE_FILE, 
                             matrix=matrix,
                             pixels_per_meter=PIXELS_PER_METER,
                             bev_width=BEV_WIDTH,
                             bev_height=BEV_HEIGHT,
                             robot_x=ROBOT_POS_X,
                             robot_y=ROBOT_POS_Y)
                    
                    print(f"Calibration saved to {SAVE_FILE}")
                    break
                else:
                    print("Requires 4 points to save")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()