import cv2
import numpy as np
import pyrealsense2 as rs
import os
import atexit
import math
import time

# ======Imports======
from RedLiveDetection import transform_to_bev, detect_red_center
import sabertooth as st
from PID import PID

# =======Config======
LOAD_FILE = r'calibration_data.npz'

#controller constants
MAX_ACUTUATOR_INPUT = 30 
S_MAX = (MAX_ACUTUATOR_INPUT - 10) * 0.6
GOAL_THRESH = 0.3  #meters
ANGLE_THRESH = np.deg2rad(3)

#robot physical constants (meters)
R_wheel = 0.08  
L = 0.178       

#gains
K_e = 30
K_theta = -K_e * 10 

def main():
    #1) Load calibration data
    if not os.path.exists(LOAD_FILE):
        print(f'Error: Calibration file not found at {LOAD_FILE}')
        return

    print(f'Loading calibration from {LOAD_FILE}')
    data = np.load(LOAD_FILE)
    matrix = data['matrix']
    pixels_per_meter = float(data['pixels_per_meter'])
    bev_width = int(data['bev_width'])
    bev_height = int(data['bev_height'])
    robot_x = int(data['robot_x'])
    robot_y = int(data['robot_y'])

    #2) Initialize camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)

    #3)Initialize motors
    print('Initializing Motors')
    try:
        motor = st.SaberToothMotorDriver(True, True)
    except Exception as e:
        print(f'Failed to initialize motors: {e}')
        return
    atexit.register(motor.all_motors_off)

    #4) Initialize PID controller for heading
    # Ts=0.033 assumes ~30FPS from the camera.
    pid_heading = PID(
        Kp=K_theta, 
        Ki=0.1,
        Kd=0.05,
        Ts=1/30, 
        umin=-200, 
        umax=200
    )

    print('RUNNING PID GO-TO-GOAL CONTROL')
    print('Press ESC to Quit.')

    try:
        while True:
            # ======Machine Vision=====
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            frame = np.asanyarray(color_frame.get_data())

            #transform and detect
            bev_image = transform_to_bev(frame, matrix, bev_width, bev_height)
            target_center = detect_red_center(bev_image)

            #draw robot reference
            cv2.circle(bev_image, (robot_x, robot_y), 10, (255, 0, 0), -1)

            # ======Control=======
            if target_center:
                cX, cY = target_center
                cv2.circle(bev_image, (cX, cY), 7, (0, 255, 0), -1)

                #1) Calculate error vector
                #calculate pixel difference
                dx_px = cX - robot_x
                dy_px = robot_y - cY 
                
                #convert to meters
                rel_x = dx_px / pixels_per_meter
                rel_y = dy_px / pixels_per_meter

                rel_x -= 0.1 #meter offset of auger from camera
                rel_y += 0.16 #meter offset of auger from camera fov

                #map to robot frame (X=forward, Y=left)
                x_robot = rel_y
                y_robot = -rel_x
                
                dist_to_goal = math.sqrt(x_robot**2 + y_robot**2)

                if dist_to_goal > GOAL_THRESH:
                    #2) Linear velocity control
                    Ux_des = K_e * x_robot
                    Uy_des = K_e * y_robot

                    #desired heading calculation
                    theta_des = math.atan2(Uy_des, Ux_des)
                    
                    #arap Angle (-pi to pi)
                    theta_des_wrapped = math.atan2(math.sin(theta_des), math.cos(theta_des))

                    #3) Angular velocity control
                    w_des = pid_heading.update(theta_des_wrapped, 0)
                    
                    #apply angle threshold
                    #if error is tiny, stop rotating to prevent jitter
                    if abs(theta_des_wrapped) < ANGLE_THRESH:
                        w_des = 0

                    #4) Kinematics
                    S_des = math.sqrt(Ux_des**2 + Uy_des**2)
                    S_sat = np.clip(S_des, -S_MAX, S_MAX)

                    wr_des = (S_sat - L * w_des) / R_wheel
                    wl_des = (S_sat + L * w_des) / R_wheel

                    #saturation and scaling
                    maxInput = max(abs(wr_des), abs(wl_des))
                    if maxInput > MAX_ACUTUATOR_INPUT:
                        scale = MAX_ACUTUATOR_INPUT / maxInput
                        wr_des_sat = wr_des * scale
                        wl_des_sat = wl_des * scale
                    else:
                        wr_des_sat = wr_des
                        wl_des_sat = wl_des

                    motor.updateMotorSpeed(wl_des_sat, wr_des_sat)
                    
                    #debug text
                    label = f'L:{wl_des_sat:.0f} R:{wr_des_sat:.0f} Err:{np.rad2deg(theta_des_wrapped):.1f}'
                    cv2.putText(bev_image, label, (cX, cY - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                else:
                    print('Goal Reached')
                    motor.updateMotorSpeed(0, 0)
            else:
                motor.updateMotorSpeed(0, 0)

            cv2.imshow('Robot Controller View', bev_image)
            
            key = cv2.waitKey(1)
            if key == 27: break

    finally:
        print('Stopping...')
        motor.all_motors_off()
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()