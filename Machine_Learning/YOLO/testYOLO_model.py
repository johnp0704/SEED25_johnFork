'''
John Poirier
SEED 485: The AWGR Autonomous Weeding Garden Robot
'''

'''
This file loads a trained YOLO model to perform binary image classification:
- Either there is a dandelion in the image or there isn't.
It then compares predictions against known YOLO label files and reports accuracy, precision, and recall.
'''

from ultralytics import YOLO
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt
import os

#============Config==================
# Path to trained YOLO model (adjust as needed)
model_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\YOLO\runs\dandelion_train_v3\weights\best.pt"

# Path to test image and label folders (adjust as needed)
image_folder = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\YOLOTestingAnnotations\Data\images\test"
label_folder = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\YOLOTestingAnnotations\Data\labels\test"

# Load trained YOLO model
model = YOLO(model_path)
print(f'Loaded YOLO model from:\n{model_path}\n')

#==============Helper Functions=================
def get_prediction(model, image_path):
    '''Runs YOLO inference on a single image and returns 1 if any detections, else 0.'''
    results = model(image_path, verbose=False)
    detections = results[0].boxes
    return 1 if len(detections) > 0 else 0


def get_ground_truth(image_folder, label_folder):
    '''Build a dictionary mapping image paths to binary ground-truth labels.'''
    ground_truth = {}
    for root, _, filenames in os.walk(image_folder):
        for filename in filenames:
            if not filename.lower().endswith('.png'):
                continue

            image_path = os.path.join(root, filename)
            label_name = os.path.splitext(filename)[0] + '.txt'
            label_path = os.path.join(label_folder, label_name)

            # A non-empty label file = dandelion present
            if os.path.exists(label_path) and os.path.getsize(label_path) > 0:
                ground_truth[image_path] = 1
            else:
                ground_truth[image_path] = 0

    return ground_truth


#===============Evaluate=====================
ground_truths = get_ground_truth(image_folder, label_folder)

y_true = []
y_pred = []
TP = TN = FP = FN = 0

for image_path, actual in ground_truths.items():
    pred = get_prediction(model, image_path)
    y_true.append(actual)
    y_pred.append(pred)

    if pred == 1 and actual == 1:
        TP += 1
    elif pred == 0 and actual == 0:
        TN += 1
    elif pred == 1 and actual == 0:
        FP += 1
    elif pred == 0 and actual == 1:
        FN += 1

#===============Metrics and Confusion Matrix================
accuracy = (TP + TN) / (TP + TN + FP + FN)
precision = TP / (TP + FP) if (TP + FP) > 0 else 0
recall = TP / (TP + FN) if (TP + FN) > 0 else 0

print('Evaluation Complete')
print(f'Accuracy:  {accuracy:.2%}')
print(f'Precision: {precision:.2%}')
print(f'Recall:    {recall:.2%}')
print(f'TP: {TP}, TN: {TN}, FP: {FP}, FN: {FN}')

cm = confusion_matrix(y_true, y_pred)
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=['No Dandelion', 'Dandelion'])
disp.plot(cmap=plt.cm.Blues)
plt.title('YOLO Confusion Matrix')
plt.show()

cm = confusion_matrix(y_true, y_pred, normalize='true')
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=['No Dandelion', 'Dandelion'])
disp.plot(cmap=plt.cm.Blues)
plt.title('YOLO Confusion Matrix')
plt.show()
