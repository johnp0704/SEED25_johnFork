import os
import json
import numpy as np
import matplotlib.pyplot as plt

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


# Load test dataset from .txt files
def load_test_data(paths_file, labels_file):
    with open(paths_file, "r") as f:
        img_paths = [line.strip() for line in f.readlines()]

    with open(labels_file, "r") as f:
        labels = [int(line.strip()) for line in f.readlines()]

    if len(img_paths) != len(labels):
        raise ValueError("Image path count does not match label count!")

    dataset = [(img_paths[i], labels[i]) for i in range(len(img_paths))]
    return dataset


# Evaluate model
def evaluate(model_path, paths_file, labels_file):

    booster = xgb.Booster()
    booster.load_model(model_path)
    model = BoosterWrapper(booster)

    dataset = load_test_data(paths_file, labels_file)

    y_true = []
    y_pred = []

    print(f"\nLoaded {len(dataset)} test images.")

    for img_path, label in dataset:

        if not os.path.exists(img_path):
            print(f"WARNING: Missing image: {img_path}")
            continue

        feat = extract_feature(img_path)
        pred = model.predict(feat)[0]

        y_true.append(label)
        y_pred.append(pred)

    print("\nXGBoost Test Results:")
    print(classification_report(y_true, y_pred, digits=4))

    #confusion martrices
    cm_raw = confusion_matrix(y_true, y_pred, labels=[0, 1])

    print("\nRaw Confusion Matrix:")
    print(cm_raw)

    disp_raw = ConfusionMatrixDisplay(cm_raw, display_labels=["No Dandelion", "Dandelion"])
    disp_raw.plot(cmap=plt.cm.Purples)
    plt.title("XGBoost Confusion Matrix (Raw Counts)")
    plt.show()

    cm_norm = confusion_matrix(y_true, y_pred, labels=[0, 1], normalize="true")

    print("\nNormalized Confusion Matrix:")
    print(cm_norm)

    disp_norm = ConfusionMatrixDisplay(cm_norm, display_labels=["No Dandelion", "Dandelion"])
    disp_norm.plot(cmap=plt.cm.Purples)
    plt.title("XGBoost Confusion Matrix (Normalized)")
    plt.show()

    return cm_raw, cm_norm



# Run evaluation
if __name__ == "__main__":

    model_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\XGBoost\xgb_model.json"

    paths_file = r"C:\UVM\SEED25_johnFork\Machine_Learning\XGBoost\test_image_paths.txt"
    labels_file = r"C:\UVM\SEED25_johnFork\Machine_Learning\XGBoost\test_labels.txt"

    evaluate(model_path, paths_file, labels_file)
