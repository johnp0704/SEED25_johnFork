'''
John Poirier
SEED 485: The AWGR Autonomous Weeding Garden Robot
'''

'''Very simple binary classification example; given annotated training images, and given test images, classify if the image
has red. Does not worry about where or how many. This file is just the model training.'''

#Make sure to install ultralytics
from ultralytics import YOLO

def main():
    # Load a pretrained YOLO model
    model = YOLO("yolov8n.pt") 

    # Train it on your data
    model.train(
        data=r"C:\UVM\SEED25\Machine_Learning\YOLOred\red.yaml", 
        epochs=30,
        imgsz=1280,
        batch=-1, 
        project=r"C:\UVM\SEED25\Machine_Learning\YOLOred\runs", 
        name="red_train_", 
        workers=8 # You can adjust this if you still get memory errors
    )

if __name__ == '__main__':
    # This prevents the code from running when imported by subprocesses
    main()