import cv2
import cv2.aruco as aruco
import numpy as np

def generate_markers():
    # Load the 4x4 dictionary (perfect for close-to-mid range tracking)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    # Generate 4 markers for the 4 walls (IDs 0, 1, 2, 3)
    marker_size = 400 # Pixels
    
    for marker_id in range(4):
        marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        filename = f"aruco_wall_{marker_id}.png"
        cv2.imwrite(filename, marker_img)
        print(f"Saved {filename}")

if __name__ == "__main__":
    generate_markers()