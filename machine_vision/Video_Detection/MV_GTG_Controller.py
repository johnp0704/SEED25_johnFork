import cv2
import numpy as np
import pyrealsense2 as rs
import os
import atexit
import math
import time

# === Imports ===
from RedLiveDetection import transform_to_bev, detect_red_center
import sabertooth as st
from PID import PID 

# === Configuration ===
LOAD_FILE = r"calibration_data.npz"

# Controller Constants
MAX_ACUTUATOR_INPUT = 30 
S_MAX = (MAX_ACUTUATOR_INPUT - 10) * 0.6
GOAL_THRESH = 0.3  # meters
ANGLE_THRESH = np.deg2rad(3)   # Deadband: Stop rotating if error is smaller than this
PIVOT_THRESHOLD = np.deg2rad(15) # Pivot: If error > 15 deg, spin in place first

# Robot Physical Constants (Meters)
R_wheel = 0.08  
L = 0.178       

# Gains
K_e = 30
K_theta = -K_e * 10 

def main():
    # 1. Load Calibration Data
    if not os.path.exists(LOAD_FILE):
        print(f"Error: Calibration file not found at {LOAD_FILE}")
        return

    data = np.load(LOAD_FILE)
    matrix = data['matrix']
    pixels_per_meter = float(data['pixels_per_meter'])
    bev_width = int(data['bev_width'])
    bev_height = int(data['bev_height'])
    robot_x = int(data['robot_x'])
    robot_y = int(data['robot_y'])

    # 2. Initialize Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)

    # 3. Initialize Motors
    print("Initializing Motors...")
    try:
        motor = st.SaberToothMotorDriver(True, True)
    except Exception as e:
        print(f"Failed to initialize motors: {e}")
        return
    atexit.register(motor.all_motors_off)

    # 4. Initialize PID for Heading
    # Increased umax/umin slightly to ensure it has enough power to pivot in place
    pid_heading = PID(
        Kp=K_theta, 
        Ki=0.0,      
        Kd=0.0,      
        Ts=1/30, 
        umin=-200, 
        umax=200
    )

    print("RUNNING PIVOT-FIRST CONTROLLER")
    print(f"Pivot Threshold: {np.rad2deg(PIVOT_THRESHOLD):.1f} degrees")
    print("Press 'ESC' to Quit.")

    try:
        while True:
            # === Vision Step ===
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            frame = np.asanyarray(color_frame.get_data())
            bev_image = transform_to_bev(frame, matrix, bev_width, bev_height)
            target_center = detect_red_center(bev_image)

            # Robot reference
            cv2.circle(bev_image, (robot_x, robot_y), 10, (255, 0, 0), -1)

            # === Control Step ===
            if target_center:
                cX, cY = target_center
                cv2.circle(bev_image, (cX, cY), 7, (0, 255, 0), -1)

                # -- 1. Calculate Error Vector --
                dx_px = cX - robot_x
                dy_px = robot_y - cY 
                
                rel_x = dx_px / pixels_per_meter
                rel_y = dy_px / pixels_per_meter

                rel_x += 0.1 #Tuning for auger positioning
                rel_y -= 0.16

                # Robot Frame: X=Forward, Y=Left
                x_robot = rel_y
                y_robot = -rel_x
                
                dist_to_goal = math.sqrt(x_robot**2 + y_robot**2)

                if dist_to_goal > GOAL_THRESH:
                    # -- 2. Calculate Desired Heading --
                    theta_des = math.atan2(K_e * y_robot, K_e * x_robot)
                    theta_des_wrapped = math.atan2(math.sin(theta_des), math.cos(theta_des))

                    # -- 3. Determine Motion Mode (Pivot vs Drive) --
                    # If the heading error is large, we kill forward momentum (S_sat = 0)
                    # This forces the math below to result in pure rotation (one wheel fwd, one back)
                    if abs(theta_des_wrapped) > PIVOT_THRESHOLD:
                        mode = "PIVOT"
                        S_sat = 0
                    else:
                        mode = "DRIVE"
                        # Standard P-control for forward velocity
                        Ux_des = K_e * x_robot
                        Uy_des = K_e * y_robot
                        S_des = math.sqrt(Ux_des**2 + Uy_des**2)
                        S_sat = np.clip(S_des, -S_MAX, S_MAX)

                    # -- 4. Angular PID Update --
                    w_des = pid_heading.update(theta_des_wrapped, 0)
                    
                    # Deadband check to stop jittering when aligned
                    if abs(theta_des_wrapped) < ANGLE_THRESH:
                        w_des = 0

                    # -- 5. Kinematics --
                    # If S_sat is 0, these become equal and opposite -> Pivot
                    wr_des = (S_sat - L * w_des) / R_wheel
                    wl_des = (S_sat + L * w_des) / R_wheel

                    # Saturation
                    maxInput = max(abs(wr_des), abs(wl_des))
                    if maxInput > MAX_ACUTUATOR_INPUT:
                        scale = MAX_ACUTUATOR_INPUT / maxInput
                        wr_des_sat = wr_des * scale
                        wl_des_sat = wl_des * scale
                    else:
                        wr_des_sat = wr_des
                        wl_des_sat = wl_des

                    motor.updateMotorSpeed(wl_des_sat, wr_des_sat)
                    
                    # Debug Text
                    deg_err = np.rad2deg(theta_des_wrapped)
                    label = f"[{mode}] Err:{deg_err:.0f}deg"
                    cv2.putText(bev_image, label, (cX, cY - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                else:
                    print("Goal Reached")
                    motor.updateMotorSpeed(0, 0)
            else:
                motor.updateMotorSpeed(0, 0)

            cv2.imshow("Robot Controller View", bev_image)
            
            key = cv2.waitKey(1)
            if key == 27: break

    finally:
        print("Stopping...")
        motor.all_motors_off()
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()