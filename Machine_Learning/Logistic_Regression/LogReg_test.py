'''
John Poirier
Logistic Regression Test
'''

import os
import cv2
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt
import pickle

# =================Config=================
IMG_SIZE = (64, 64)
OUTPUT_DIR = r"C:\UVM\SEED25_johnFork\Machine_Learning\Logistic_Regression"

# Flat directory containing all annotation text files
LR_DATA_ROOT = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\LogRegannotations"

os.makedirs(OUTPUT_DIR, exist_ok=True)

# =================Helper Functions=================
def load_lr_dataset(split):
    """
    Loads image paths and labels from text files located in a flat directory.
    Expects files like:
        train_image_paths.txt
        train_binary_labels.txt
        test_image_paths.txt
        test_binary_labels.txt
        val_image_paths.txt
        val_binary_labels.txt
    """
    
    image_paths_file = os.path.join(LR_DATA_ROOT, f"{split}_image_paths.txt")
    label_paths_file = os.path.join(LR_DATA_ROOT, f"{split}_binary_labels.txt")

    if not os.path.exists(image_paths_file) or not os.path.exists(label_paths_file):
        raise FileNotFoundError(
            f"Missing required files: {image_paths_file} OR {label_paths_file}"
        )

    # Read image paths
    with open(image_paths_file, 'r') as f:
        image_paths = [p.strip() for p in f.readlines()]

    # Read labels
    with open(label_paths_file, 'r') as f:
        y = np.array([int(l.strip()) for l in f.readlines()])

    X = []
    print(f"Loading {len(image_paths)} images for {split} set.")

    for img_path in image_paths:
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not read {img_path}. Skipping.")
            continue

        img_resized = cv2.resize(img, IMG_SIZE)
        X.append(img_resized.flatten())

    return np.array(X), y

# =================Main=================
if __name__ == "__main__":
    # Load datasets
    X_train, y_train = load_lr_dataset("train")
    X_test,  y_test  = load_lr_dataset("test")

    print(f"\nTraining Data Shape: {X_train.shape}")
    print(f"Testing Data Shape:  {X_test.shape}")

    # Scale data
    print("\nScaling data...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    # Train model
    print("Training Logistic Regression...")
    model = LogisticRegression(max_iter=1000, solver='lbfgs', verbose=1)
    model.fit(X_train_scaled, y_train)

    # Save model + scaler
    with open(os.path.join(OUTPUT_DIR, "logreg_model.pkl"), "wb") as f:
        pickle.dump(model, f)
    with open(os.path.join(OUTPUT_DIR, "scaler.pkl"), "wb") as f:
        pickle.dump(scaler, f)

    print(f"Model and scaler saved to {OUTPUT_DIR}")

    # Evaluate
    print("\nEvaluating...")
    y_pred = model.predict(X_test_scaled)

    print("\nClassification Report:")
    print(classification_report(y_test, y_pred,
                                target_names=["No Dandelion", "Dandelion"]))

    # Confusion Matrix
    cm = confusion_matrix(y_test, y_pred)
    disp = ConfusionMatrixDisplay(confusion_matrix=cm,
                                  display_labels=['No Dandelion', 'Dandelion'])
    disp.plot(cmap=plt.cm.Reds)
    plt.title('Logistic Regression Confusion Matrix')
    plt.show()

    #normalized cm
    cm = confusion_matrix(y_test, y_pred, normalize='true')
    disp = ConfusionMatrixDisplay(confusion_matrix=cm,
                                  display_labels=['No Dandelion', 'Dandelion'])
    disp.plot(cmap=plt.cm.Reds)
    plt.title('Logistic Regression Confusion Matrix')
    plt.show()