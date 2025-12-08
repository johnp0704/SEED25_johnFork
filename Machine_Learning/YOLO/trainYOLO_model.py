'''
John Poirier
SEED 485: The AWGR Autonomous Weeding Garden Robot
'''

#Make sure to install ultralytics
from ultralytics import YOLO

if __name__ == '__main__': #required so worker scripts don't run this when called
    #load a pretrained YOLO model
    model = YOLO("yolov8n.pt")  # or "yolo11n.pt" if you runnin YOLOv11, have to delete each time you train? I guess?

    #train it on your data
    model.train(
        data=r"C:\UVM\SEED25_johnFork\Machine_Learning\YOLO\dandelion.yaml", #gotta change to your local repo path
        epochs=30,
        imgsz=1280,
        batch=-1, #autotune batch size based on GPU memory
        project=r"C:\UVM\SEED25_johnFork\Machine_Learning\YOLO\runs",  #base folder
        name="dandelion_train_v",  #subfolder for this specific run
        device = '0' #can change if you have multiple GPUs
    )

