# Examples for interfacing with the Jetson 

## circles_test.py
Follow this section to run the Dynamic Window Approach to collision avoidance, with object detection from a standard package. In other words, this program will detect an object, find the middle of the bounding box, and navigate around it. There are no motor commands here as this is a purely visual aid.

You will need to pip install the following packages before running the code in python:
1. pip install opencv-python
2. pip install numpy
3. pip install matplotlib
4. pip install pyrealsense2
5. pip install imutils

On top of this, there are two ways to get the Realsense to work with python. The first is as mentioned above by "pip install pyrealsense2", but may require a virtual environment. The second involves following this github instruction set for the jetson. Follow [this link](https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation) closely to install the Realsense packages natively.

From here, plug in the Realsense via USB and run the python circles_test.py script.

## BirdsEyeView.py
Follow this section to run a Birds Eye View transformation for a given image. This can be paired with the circles_test.py script above to perform a more precise DWA calculation, as this is a direct top-down measurement, rather than an extrapolated one. In other words, when we command a robot, we usually tell it how to move from a top-down reference frame, however we are taking a front-facing image with the Realsense. To get the most accurate measurements and hence the most accurate commands, use this transformation. Again, this program does not use motor commands as this is a purely visual aid.

You will need to pip install the following packages before running the code in python:
1. pip install opencv-python
2. pip install numpy
3. pip install matplotlib
4. pip install pyrealsense2
5. pip install imutils
6. pip install python-math

On top of this, there are two ways to get the Realsense to work with python. The first is as mentioned above by "pip install pyrealsense2", but may require a virtual environment. The second involves following this github instruction set for the jetson. Follow [this link](https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation) closely to install the Realsense packages natively.

From here, plug in the Realsense via USB and run the python BirdsEyeView.py script. There is a calibration board in the V308 closet if you need to change parameters to get a perfect top-down perspective.

## motor_test.py
Follow this section to gain an understanding of how to command the motors using Simple Serial Mode. The script provided cycles the speeds of the left and right motors through a for loop.

You will need to pip install the following packages before running the code in python:
1. pip install pyserial
2. the "time" module is used, but is already part of python's standard packages so this does not need to be pip'd

Using the Sabertooth 2x12 datasheet, set to Simple Serial and the baud rate to 9600 by adjusting the dip switches. Prop the robot up on a box so that the wheels do not touch the ground, otherwise it will run away when the script is ran! Here no sensors are used, this is used for testing the motors and seeing how commands are sent.
