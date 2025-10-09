from ultralytics import YOLO

model = YOLO('yolov8n.pt')
model.train(
        data='/home/airlab/realsense/Stop Sign.v2i.yolov8/data.yaml',
        epochs=50,
        imgsz=640,
        batch=16,
        workers=4
    )
