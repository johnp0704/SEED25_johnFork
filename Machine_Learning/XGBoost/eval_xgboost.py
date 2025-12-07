import os
import json
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET

from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay, classification_report
import xgboost as xgb

from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.utils import load_img, img_to_array


# Load Resnet50 model for feature extraction
IMG_SIZE = (224, 224)
resnet_model = ResNet50(weights="imagenet", include_top=False, pooling="avg")

def extract_feature(img_path):
    img = load_img(img_path, target_size=IMG_SIZE)
    arr = img_to_array(img)
    arr = np.expand_dims(arr, axis=0)
    arr = preprocess_input(arr)
    feat = resnet_model.predict(arr, verbose=0)[0]
    return feat.reshape(1, -1)

# XGBoost Booster Wrapper
class BoosterWrapper:
    def __init__(self, booster):
        self.booster = booster

    def predict_proba(self, X):
        dX = xgb.DMatrix(X)
        probs = self.booster.predict(dX)
        return np.column_stack([1 - probs, probs])

    def predict(self, X):
        probs = self.predict_proba(X)[:, 1]
        return (probs >= 0.5).astype(int)


# Parse annotations
def parse_cvat_xml(xml_path, image_dir):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    data = []

    for img_tag in root.findall("image"):
        filename = img_tag.get("name")

        has_obj = any([
            len(img_tag.findall("box")),
            len(img_tag.findall("polygon")),
            len(img_tag.findall("polyline")),
            len(img_tag.findall("points")),
        ])

        label = 1 if has_obj else 0
        full_path = os.path.join(image_dir, filename)

        if not os.path.exists(full_path):
            print(f"WARNING: Image missing {full_path}")
            continue

        data.append((full_path, label))

    return data


# Evaluate model
def evaluate(model_path, xml_path, img_dir):

    # Booster
    booster = xgb.Booster()
    booster.load_model(model_path)
    model = BoosterWrapper(booster)

    # Validation data
    dataset = parse_cvat_xml(xml_path, img_dir)

    y_true = []
    y_pred = []

    print(f"\nLoaded {len(dataset)} images from validation set.")

    # Prediction
    for img_path, label in dataset:
        feat = extract_feature(img_path)     # shape (1, 2048)
        pred = model.predict(feat)[0]

        y_true.append(label)
        y_pred.append(pred)

    print("\nXGBoost Validation Results")
    print(classification_report(y_true, y_pred, digits=4))

    # Confusion Matrix
    cm = confusion_matrix(y_true, y_pred, labels=[0, 1])
    labels = ["No Dandelion", "Dandelion"]

    fig, ax = plt.subplots(figsize=(6, 6))
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=labels)

    disp.plot(
        cmap="Greens",
        colorbar=True,
        values_format="d",
        ax=ax
    )

    ax.set_title("XGBoost Confusion Matrix — Validation Set", fontsize=16, pad=20)
    ax.set_xlabel("Predicted Label", fontsize=14)
    ax.set_ylabel("True Label", fontsize=14)
    ax.set_aspect("equal")

    for spine in ax.spines.values():
        spine.set_linewidth(1.5)

    plt.tight_layout()
    plt.show()

    print("\nConfusion Matrix:")
    print(cm)

    return cm


# Run evaluation
if __name__ == "__main__":

    model_path = r"C:\Users\samst\Downloads\xgboost_model.json"

    xml_path = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 5\annotations.xml"

    img_dir = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 5"

    evaluate(model_path, xml_path, img_dir)
