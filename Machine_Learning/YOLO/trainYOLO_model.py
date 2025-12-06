'''
John Poirier
SEED 485: The AWGR Autonomous Weeding Garden Robot
'''

'''Very simple binary classification example; given 100 annotated training images (dandelion wave 1), and given 1 new test image, classify if the image
has a dandelion. Does not worry about where or how many. This file is just the model training.'''

#Make sure to install ultralytics
from ultralytics import YOLO

# Load a pretrained YOLO model
model = YOLO("yolov8n.pt")  # or "yolo11n.pt" if you runnin YOLOv11, have to delete each time you train? I guess?

# Train it on your data
model.train(
    data=r"C:\UVM\SEED25_johnFork\Machine_Learning\YOLO\dandelion.yaml", #gotta change to your local repo path
    epochs=30,
    imgsz=1280,
    batch=-1, #autotune batch size based on GPU memory
    project=r"C:\UVM\SEED25_johnFork\Machine_Learning\YOLO\runs",  #base folder
    name="dandelion_train_v1",  #subfolder for this specific run
    device = '0' #can change if you have multiple GPUs
)

