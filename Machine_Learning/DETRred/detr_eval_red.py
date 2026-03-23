import torch
from transformers import DetrConfig, DetrForObjectDetection, DetrImageProcessor
from PIL import Image
import json
import os
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt

def evaluate_model(model_path, val_img_dir, val_ann_file, threshold=0.05):

    # -----------------------------
    # LOAD MODEL (MANUAL OVERRIDE)
    # -----------------------------
    print(f"Loading model configuration from {model_path}...")
    # 1. Load the configuration only
    config = DetrConfig.from_pretrained(model_path)
    
    # 2. Instantiate the model structure (Forces generic CPU initialization)
    model = DetrForObjectDetection(config)
    
    # 3. Manually load the weights
    # Check if the model uses safetensors or standard pytorch bin
    bin_path = os.path.join(model_path, "pytorch_model.bin")
    safetensors_path = os.path.join(model_path, "model.safetensors")
    
    if os.path.exists(safetensors_path):
        from safetensors.torch import load_file
        state_dict = load_file(safetensors_path)
        print("Loaded weights from model.safetensors")
    elif os.path.exists(bin_path):
        state_dict = torch.load(bin_path, map_location="cpu")
        print("Loaded weights from pytorch_model.bin")
    else:
        raise FileNotFoundError(f"Could not find model weights (pytorch_model.bin or model.safetensors) in {model_path}")

    # Load the state dict into the model
    # strict=False allows for minor header mismatches, but usually we want True to ensure integrity
    missing_keys, unexpected_keys = model.load_state_dict(state_dict, strict=False)
    
    if len(missing_keys) > 0:
        print(f"WARNING: Missing keys during load: {missing_keys}")
    
    # Load processor
    processor = DetrImageProcessor.from_pretrained(model_path)
    
    model.to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()

    # -----------------------------
    # LOAD COCO ANNOTATIONS
    # -----------------------------
    with open(val_ann_file, "r") as f:
        coco = json.load(f)

    # For storing predicted + true binary labels
    y_true = []
    y_pred = []

    # Map image IDs -> annotations
    img_id_to_anns = {img["id"]: [] for img in coco["images"]}
    for ann in coco["annotations"]:
        img_id_to_anns[ann["image_id"]].append(ann)

    # -----------------------------
    # RUN INFERENCE ON ALL IMAGES
    # -----------------------------
    print(f"Running inference on {len(coco['images'])} images...")
    for img_info in coco["images"]:
        img_id = img_info["id"]
        filename = img_info["file_name"]
        image_path = os.path.join(val_img_dir, filename)
        
        try:
            image = Image.open(image_path).convert("RGB")
        except Exception as e:
            print(f"Could not open {image_path}: {e}")
            continue

        # Ground truth label
        gt_has_dandelion = len(img_id_to_anns[img_id]) > 0
        y_true.append(1 if gt_has_dandelion else 0)

        # Model prediction
        inputs = processor(images=image, return_tensors="pt").to(model.device)
        with torch.no_grad():
            outputs = model(**inputs)

        # Post process outputs
        results = processor.post_process_object_detection(
            outputs,
            target_sizes=[image.size[::-1]],
            threshold=threshold
        )[0]

        pred_has_dandelion = len(results["boxes"]) > 0
        y_pred.append(1 if pred_has_dandelion else 0)

    # -----------------------------
    # CONFUSION MATRIX + METRICS
    # -----------------------------
    cm = confusion_matrix(y_true, y_pred, labels=[0, 1])
    TN, FP, FN, TP = cm.ravel()

    # Calculate metrics, handling division by zero
    accuracy = (TP + TN) / (TP + TN + FP + FN) if (TP + TN + FP + FN) > 0 else 0
    precision = TP / (TP + FP) if (TP + FP) > 0 else 0
    recall = TP / (TP + FN) if (TP + FN) > 0 else 0

    # Plot confusion matrix
    labels = ["No Red", "Red"]
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=labels)
    disp.plot(cmap="Greens")
    plt.title("DETR Image-Level Confusion Matrix")
    plt.show()

    # Print results
    print("\nConfusion Matrix:")
    print(cm)

    print("\nDETR Evaluation Metrics")
    print(f"Accuracy:  {accuracy:.2%}")
    print(f"Precision: {precision:.2%}")
    print(f"Recall:    {recall:.2%}")
    print(f"TP: {TP}, TN: {TN}, FP: {FP}, FN: {FN}")

    return cm


# ----------------------------------------------------------
# RUN THE SCRIPT
# ----------------------------------------------------------
if __name__ == "__main__":
    model_path = r"C:\UVM\SEED\SEED25\Machine_Learning\DETRred\detr_red_model"
    val_img_dir = r"C:\UVM\SEED\SEED25\Images\Testing Annotated\DETRredannotations\red_dataset_detr\test\images"
    val_ann_file = r"C:\UVM\SEED\SEED25\Images\Testing Annotated\DETRredannotations\red_dataset_detr\annotations\instances_test.json"

    evaluate_model(model_path, val_img_dir, val_ann_file)

