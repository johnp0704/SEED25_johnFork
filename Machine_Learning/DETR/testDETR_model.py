import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image
import json
import os
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt

#============Config==================
# Directory containing config.json and model.safetensors
model_path = r"C:\UVM\SEED\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_best"

# COCO formatted data
test_img_dir = r"C:\UVM\SEED\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\test\images"
test_ann_file = r"C:\UVM\SEED\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_test.json"

# Global detection threshold
THRESHOLD = 0.9

#==============Helper Functions=================
def get_prediction(image_path, model, processor, threshold):
    '''Runs DETR inference on a single image and returns 1 if detections found above threshold.'''
    image = Image.open(image_path).convert("RGB")
    inputs = processor(images=image, return_tensors="pt").to(model.device)
    
    with torch.no_grad():
        outputs = model(**inputs)

    # Convert results back to image scale to extract bounding boxes
    results = processor.post_process_object_detection(
        outputs, 
        target_sizes=[image.size[::-1]], 
        threshold=threshold
    )[0]

    #debuggin: show detection probabilities
    with torch.no_grad():
        outputs = model(**inputs)

    # View top probability for query 0
    probs = outputs.logits.softmax(-1)
    # Grab the highest prob for any of the 100 queries, excluding the last class (background)
    max_val, max_idx = probs[0, :, :-1].max(-1) 
    print(f"Max detection probability found in image: {max_val.max().item():.4f}")

    return 1 if len(results["boxes"]) > 0 else 0

def load_coco_ground_truth(val_ann_file):
    '''Loads COCO JSON and returns a mapping of image filenames to binary labels.'''
    with open(val_ann_file, "r") as f:
        coco = json.load(f)

    # Initialize dict with 0 (no dandelion)
    ground_truth = {img["file_name"]: 0 for img in coco["images"]}
    
    # Update to 1 if there is an annotation linked to that image_id
    id_to_filename = {img["id"]: img["file_name"] for img in coco["images"]}
    for ann in coco["annotations"]:
        filename = id_to_filename[ann["image_id"]]
        ground_truth[filename] = 1
        
    return ground_truth

#===============Evaluate=====================
def main():
    # Load model and processor
    processor = DetrImageProcessor.from_pretrained(model_path)
    model = DetrForObjectDetection.from_pretrained(model_path)

    #ensure classification ids match
    model.config.id2label = {0: "dandelion"}
    model.config.label2id = {"dandelion": 0}

    model.to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()
    print(f'Loaded DETR model from:\n{model_path}\n')

    # Load data
    ground_truths = load_coco_ground_truth(test_ann_file)

    y_true = []
    y_pred = []
    TP = TN = FP = FN = 0

    print("Evaluating model...")
    for filename, actual in ground_truths.items():
        image_path = os.path.join(test_img_dir, filename)
        
        # Check if file exists to prevent errors
        if not os.path.exists(image_path):
            continue
            
        pred = get_prediction(image_path, model, processor, THRESHOLD)
        
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
    total = TP + TN + FP + FN
    accuracy = (TP + TN) / total if total > 0 else 0
    precision = TP / (TP + FP) if (TP + FP) > 0 else 0
    recall = TP / (TP + FN) if (TP + FN) > 0 else 0

    print('\nEvaluation Complete')
    print(f'Accuracy:  {accuracy:.2%}')
    print(f'Precision: {precision:.2%}')
    print(f'Recall:    {recall:.2%}')
    print(f'TP: {TP}, TN: {TN}, FP: {FP}, FN: {FN}')

    cm = confusion_matrix(y_true, y_pred, labels=[0, 1])
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=['No Dandelion', 'Dandelion'])
    disp.plot(cmap=plt.cm.Greens)
    plt.title('DETR Confusion Matrix')
    plt.show()

    cm = confusion_matrix(y_true, y_pred, labels=[0, 1], normalize='true')
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=['No Dandelion', 'Dandelion'])
    disp.plot(cmap=plt.cm.Greens)
    plt.title('DETR Confusion Matrix')
    plt.show()

if __name__ == "__main__":
    main()