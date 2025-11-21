import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image
import json
import os
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt

def evaluate_model(model_path, val_img_dir, val_ann_file, threshold=0.5):

    # Load model
    processor = DetrImageProcessor.from_pretrained(model_path)
    model = DetrForObjectDetection.from_pretrained(model_path)
    model.to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()

    # Load COCO validation annotations
    with open(val_ann_file, "r") as f:
        coco = json.load(f)

    # ========== Ground-truth labels ==========
    # GT 1 = image has a dandelion (at least one annotation)
    # GT 0 = no dandelion
    y_true = []
    y_pred = []

    img_id_to_anns = {img["id"]: [] for img in coco["images"]}
    for ann in coco["annotations"]:
        img_id_to_anns[ann["image_id"]].append(ann)

    # Loop over every validation image
    for img_info in coco["images"]:
        img_id = img_info["id"]
        filename = img_info["file_name"]
        image_path = os.path.join(val_img_dir, filename)
        image = Image.open(image_path).convert("RGB")

        # ---------- Ground-truth ----------
        gt_has_dandelion = len(img_id_to_anns[img_id]) > 0
        y_true.append(1 if gt_has_dandelion else 0)

        # ---------- Model prediction ----------
        inputs = processor(images=image, return_tensors="pt").to(model.device)
        with torch.no_grad():
            outputs = model(**inputs)

        results = processor.post_process_object_detection(
            outputs,
            target_sizes=[image.size[::-1]],
            threshold=threshold
        )[0]

        pred_has_dandelion = len(results["boxes"]) > 0
        y_pred.append(1 if pred_has_dandelion else 0)

    # ========== Confusion Matrix ==========
    cm = confusion_matrix(y_true, y_pred)
    labels = ["No Dandelion", "Dandelion"]

    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=labels)
    disp.plot(cmap="Greens")
    plt.title("DETR Image-Level Confusion Matrix")
    plt.show()

    print("\nConfusion Matrix:")
    print(cm)

    return cm


if __name__ == "__main__":
    model_path = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Machine Learning\DETR\detr-dandelion"
    val_img_dir = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
    val_ann_file = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

    evaluate_model(model_path, val_img_dir, val_ann_file)
