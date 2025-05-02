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

From here, you will be able to plug in the Realsense via USB and run the python circles_test.py script.
