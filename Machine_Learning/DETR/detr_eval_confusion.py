import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image
import json
import os


def evaluate_model(model_path, val_img_dir, val_ann_file, threshold=0):

    # Load model
    processor = DetrImageProcessor.from_pretrained(model_path)
    model = DetrForObjectDetection.from_pretrained(model_path)
    model.to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()

    # Load COCO annotations
    with open(val_ann_file, "r") as f:
        coco = json.load(f)

    img_id_to_anns = {img["id"]: [] for img in coco["images"]}
    for ann in coco["annotations"]:
        img_id_to_anns[ann["image_id"]].append(ann)

    # Confusion matrix counters
    TP = FP = FN = TN = 0

    for img_info in coco["images"]:
        img_id = img_info["id"]
        filename = img_info["file_name"]
        image_path = os.path.join(val_img_dir, filename)

        image = Image.open(image_path).convert("RGB")

        # Ground truth
        gt_boxes = img_id_to_anns[img_id]
        gt_has_dandelion = len(gt_boxes) > 0

        # Prediction
        inputs = processor(images=image, return_tensors="pt").to(model.device)

        with torch.no_grad():
            outputs = model(**inputs)

        results = processor.post_process_object_detection(
            outputs, target_sizes=[image.size[::-1]], threshold=threshold
        )[0]

        pred_has_dandelion = len(results["boxes"]) > 0

        # Image-level confusion
        if gt_has_dandelion and pred_has_dandelion:
            TP += 1
        elif not gt_has_dandelion and pred_has_dandelion:
            FP += 1
        elif gt_has_dandelion and not pred_has_dandelion:
            FN += 1
        else:
            TN += 1

    print("\n==== Image-level Confusion Matrix ====")
    print(f"TP = {TP}")
    print(f"FP = {FP}")
    print(f"FN = {FN}")
    print(f"TN = {TN}")

    return TP, FP, FN, TN


if __name__ == "__main__":
    model_path = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Machine Learning\DETR\detr-dandelion"
    val_img_dir = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
    val_ann_file = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

    evaluate_model(model_path, val_img_dir, val_ann_file)