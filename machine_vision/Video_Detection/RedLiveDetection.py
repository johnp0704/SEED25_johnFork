import pyrealsense2 as rs
import numpy as np
import cv2
import os

#data file
LOAD_FILE = r"C:\UVM\SEED\SEED25\machine_vision\Video_Detection\calibration_data.npz"

#=====Color Configs======
LOWER_RED1 = np.array([0, 120, 70])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 120, 70])
UPPER_RED2 = np.array([180, 255, 255])

def get_red_mask(image_hsv):
    mask1 = cv2.inRange(image_hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(image_hsv, LOWER_RED2, UPPER_RED2)
    return cv2.bitwise_or(mask1, mask2)

def main():
    #load data from calibration
    if not os.path.exists(LOAD_FILE):
        print(f"Error: Calibration file not found at {LOAD_FILE}")
        return

    print(f"Loading calibration from {LOAD_FILE}...")
    data = np.load(LOAD_FILE)
    
    #get variables from data
    matrix = data['matrix']
    pixels_per_meter = float(data['pixels_per_meter'])
    bev_width = int(data['bev_width'])
    bev_height = int(data['bev_height'])
    robot_x = int(data['robot_x'])
    robot_y = int(data['robot_y'])

    #==========Initialize Camera===========
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)

    print("RUNNING BEV DETECTION")
    print("Press 'ESC' to Quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            frame = np.asanyarray(color_frame.get_data())

            #warp the image
            bev_image = cv2.warpPerspective(frame, matrix, (bev_width, bev_height))

            #detect red
            hsv_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
            red_mask = get_red_mask(hsv_bev)

            #find contours
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 50:
                    M = cv2.moments(largest)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        #draw green dot at center
                        cv2.circle(bev_image, (cX, cY), 7, (0, 255, 0), -1)

                        #calculate position relative to robot
                        dx_px = cX - robot_x
                        dy_px = robot_y - cY
                        
                        rel_x = dx_px / pixels_per_meter
                        rel_y = dy_px / pixels_per_meter

                        label = f"X:{rel_x:.2f}m Y:{rel_y:.2f}m"
                        cv2.putText(bev_image, label, (cX, cY - 15), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            #draw robot reference (will change to finalized auger position) TODO
            cv2.circle(bev_image, (robot_x, robot_y), 10, (255, 0, 0), -1)

            #display BEV Window
            cv2.imshow("Bird's Eye View", bev_image)

            # =========Keyboard Inputs=========
            key = cv2.waitKey(1)
            if key == 27: #esc
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()