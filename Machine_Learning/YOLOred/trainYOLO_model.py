'''
John Poirier
SEED 485: The AWGR Autonomous Weeding Garden Robot
'''

'''Very simple binary classification example; given annotated training images, and given test images, classify if the image
has red. Does not worry about where or how many. This file is just the model training.'''

#Make sure to install ultralytics
from ultralytics import YOLO

# Load a pretrained YOLO model
model = YOLO("yolov8n.pt")  # or "yolo11n.pt" if you runnin YOLOv11, have to delete each time you train? I guess?

# Train it on your data
model.train(
    data=r"C:\UVM\SEED\SEED25\Machine_Learning\YOLOred\red.yaml", #gotta change to your local repo path
    epochs=30,
    imgsz=1280,
    batch=-1, #autotune batch size based on GPU memory
    project=r"C:\UVM\SEED\SEED25\Machine_Learning\YOLOred\runs",  # base folder
    name="red_train_",  # subfolder for this specific run
)

