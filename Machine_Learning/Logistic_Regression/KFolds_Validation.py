"""
John Poirier
Logistic Regression K-Fold Cross-Validation (with normalized confusion matrix)
"""

import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import StratifiedKFold, cross_val_score, cross_val_predict
from sklearn.metrics import (
    classification_report,
    confusion_matrix,
    ConfusionMatrixDisplay
)
from sklearn.pipeline import Pipeline

# ============================================================
# Config (updated flat-file structure)
# ============================================================

IMG_SIZE = (64, 64)
LR_DATA_ROOT = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\LogRegannotations"

# NOTE: Files now live directly inside LR_DATA_ROOT, no subfolders.
# (E.g., train_image_paths.txt, train_binary_labels.txt, etc.)


# ============================================================
# Helper Function for New File Layout
# ============================================================

def load_lr_dataset(split):
    """
    Loads dataset using new format:
        train_image_paths.txt
        train_binary_labels.txt
        val_...
        test_...
    No subfolders.
    """

    image_paths_file = os.path.join(LR_DATA_ROOT, f"{split}_image_paths.txt")
    label_paths_file = os.path.join(LR_DATA_ROOT, f"{split}_binary_labels.txt")

    if not os.path.exists(image_paths_file) or not os.path.exists(label_paths_file):
        raise FileNotFoundError(f"Missing data files for '{split}' in LR_DATA_ROOT")

    with open(image_paths_file, 'r') as f:
        image_paths = [p.strip() for p in f.readlines()]

    with open(label_paths_file, 'r') as f:
        y = np.array([int(v.strip()) for v in f.readlines()])

    X = []
    print(f"Loading {len(image_paths)} images for '{split}'...")

    for img_path in image_paths:
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not load image: {img_path}")
            continue
        
        img_resized = cv2.resize(img, IMG_SIZE)
        X.append(img_resized.flatten())

    return np.array(X), y


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    # Load TRAIN ONLY for cross-validation
    X, y = load_lr_dataset("train")

    print(f"\nData Loaded:")
    print(f"  X shape = {X.shape}")
    print(f"  y shape = {y.shape}")

    # Pipeline ensures scaling happens inside each fold
    model = Pipeline([
        ("scaler", StandardScaler()),
        ("logreg", LogisticRegression(max_iter=1000, solver="lbfgs"))
    ])

    k = 5
    print(f"\nRunning {k}-Fold Stratified Cross-Validation...")

    skf = StratifiedKFold(n_splits=k, shuffle=True, random_state=42)

    # Cross-validation accuracy
    scores = cross_val_score(model, X, y, cv=skf, scoring="accuracy")

    print("\nCross-Val Accuracy Scores:", scores)
    print(f"Mean Accuracy: {np.mean(scores):.4f}")
    print(f"Std Dev:       {np.std(scores):.4f}\n")

    # Predictions for confusion matrices
    y_pred = cross_val_predict(model, X, y, cv=skf)

    print("Classification Report:")
    print(classification_report(y, y_pred, target_names=["No Dandelion", "Dandelion"]))

    # -------------------------------
    # Raw Confusion Matrix
    # -------------------------------
    cm_raw = confusion_matrix(y, y_pred)
    disp_raw = ConfusionMatrixDisplay(cm_raw, display_labels=["No Dandelion", "Dandelion"])
    disp_raw.plot(cmap=plt.cm.Purples)
    plt.title("K-Fold Confusion Matrix (Raw Counts)")
    plt.show()

    # -------------------------------
    # Normalized Confusion Matrix
    # -------------------------------
    cm_norm = confusion_matrix(y, y_pred, normalize="true")
    disp_norm = ConfusionMatrixDisplay(cm_norm, display_labels=["No Dandelion", "Dandelion"])
    disp_norm.plot(cmap=plt.cm.Purples)
    plt.title("K-Fold Confusion Matrix (Normalized)")
    plt.show()
