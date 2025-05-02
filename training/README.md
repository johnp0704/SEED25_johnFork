# YOLO (you only look once) v8 (version 8)
YOLO is a neural network built by Ultralytics for object detection and segmentation. 
## Instructions to compile and use 
To use YOLO, download a database of annotated images to the machine that will be used for training. Use a python script to start the training. Begin by installing the Ultralytics package through pip or any other package manager like conda. 


<pre> pip install ultralytics </pre>
Use training.py with your dataset path and run the script to train. If permission denied pops up, run it using <pre> sudo python3 training.py </pre>

The finished script will output its results to a folder within the same directory where your dataset is located. The finished script will say where that folder is. 
