# SEED 485 XGBoost Model
import numpy as np
from xgboost import XGBClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.datasets import make_classification

import os
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd

from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

import xgboost as xgb

from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.utils import load_img, img_to_array

# config

# Wave
WAVES = [
    {
        "name": "wave1",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 1",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 1"
    },
    {
        "name": "wave2",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 2",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 2"
    },
    {
        "name": "wave3",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 3",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 3"
    },
    {
        "name": "wave4",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 4",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 4"
    },
    {
        "name": "wave5",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 5",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 5"
    },
    {
        "name": "wave6",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 6",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 6"
    },
    {
        "name": "wave7",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 7",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 7"
    },
    {
        "name": "wave8",
        "xml_path": r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 8",
        "image_dir": r"C:\UVM\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 8"
    }
]

IMG_SIZE = (224, 224) # ResNet50 input size
RANDOM_STATE = 42 # for reproducible splits
FEATURE_CACHE = None


# Get image labels from CVAT XMLs

def parse_cvat_xml(xml_dir, image_dir, wave_name):
    """
    xml_dir: directory containing the CVAT XML file
    """
    # find XML file inside xml_dir
    xml_files = [f for f in os.listdir(xml_dir) if f.lower().endswith(".xml")]
    if len(xml_files) == 0:
        raise FileNotFoundError(f"No XML file found in directory: {xml_dir}")
    if len(xml_files) > 1:
        print(f"WARNING: Multiple XML files found in {xml_dir}, using the first one.")

    xml_path = os.path.join(xml_dir, xml_files[0])

    tree = ET.parse(xml_path)
    root = tree.getroot()

    """
    Parse a single CVAT 'CVAT for images 1.1' XML file.
    Returns a DataFrame with columns: ['wave', 'filename', 'full_path', 'label']
    label = 1 if image has at least one <box> (dandelion present), else 0.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    rows = []
    for img_tag in root.findall('image'):
        filename = img_tag.get('name')
        boxes = img_tag.findall('box')
        polygons = img_tag.findall('polygon')
        polylines = img_tag.findall('polyline')
        points = img_tag.findall('points')

        has_object = (
            len(boxes) > 0 or
            len(polygons) > 0 or
            len(polylines) > 0 or
            len(points) > 0
        )

        label = 1 if has_object else 0

        full_path = os.path.join(image_dir, filename)

        rows.append({
            "wave": wave_name,
            "filename": filename,
            "full_path": full_path,
            "label": label
        })

    df = pd.DataFrame(rows)
    return df


def build_labels_dataframe(waves_config):
    """
    Combine all waves into a single DataFrame.
    """
    dfs = []
    for w in waves_config:
        df_wave = parse_cvat_xml(w["xml_path"], w["image_dir"], w["name"])
        dfs.append(df_wave)

    df_all = pd.concat(dfs, ignore_index=True)
    return df_all


# CNN Feature Extraction

# Load ResNet50 once (without top classification layer)
resnet_model = ResNet50(weights='imagenet', include_top=False, pooling='avg')


def extract_feature_for_image(img_path):
    """
    Load an image from disk, preprocess for ResNet50, and return a 2048-dim feature vector.
    """
    if not os.path.exists(img_path):
        raise FileNotFoundError(f"Image not found: {img_path}")

    img = load_img(img_path, target_size=IMG_SIZE)
    x = img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)
    feat = resnet_model.predict(x, verbose=0)[0]  # shape (2048,)
    return feat


def extract_features(df, cache_path=None):
    """
    Given a DataFrame with 'full_path', return:
        X: numpy array of shape (N, feature_dim)
        y: labels array of shape (N,)
    If cache_path is provided and exists, load features from there instead of recomputing.
    """
    if cache_path is not None and os.path.exists(cache_path):
        print(f"Loading cached features from {cache_path}")
        cache = np.load(cache_path, allow_pickle=True).item()
        return cache["X"], cache["y"]

    X_list = []
    y_list = []

    for idx, row in df.iterrows():
        img_path = row['full_path']
        label = row['label']

        try:
            feat = extract_feature_for_image(img_path)
        except FileNotFoundError as e:
            print(f"WARNING: {e}")
            continue

        X_list.append(feat)
        y_list.append(label)

        if (idx + 1) % 20 == 0:
            print(f"Processed {idx + 1}/{len(df)} images...")

    X = np.array(X_list)
    y = np.array(y_list)

    if cache_path is not None:
        np.save(cache_path, {"X": X, "y": y})
        print(f"Saved features to cache: {cache_path}")

    return X, y


# 70/20/10 Split

def split_70_10_20(X, y, random_state=RANDOM_STATE):
    """
    Perform a stratified 70/20/10 split.
    1) First split into 70% train and 30% temp.
    2) Then split temp into (2/3 val, 1/3 test) → 20% / 10%.
    """
    # 1. Split off 20% test
    X_trainval, X_test, y_trainval, y_test = train_test_split(
        X, y, test_size=0.20, random_state=42, stratify=y
    )

    # 2. Split remaining into 10% val and 70% train
    # 10% of whole dataset = 0.10 / 0.80 = 0.125
    X_train, X_val, y_train, y_val = train_test_split(
        X_trainval, y_trainval, test_size=0.125, random_state=42, stratify=y_trainval
    )

    print("Split sizes:")
    print("  Train:", X_train.shape[0])
    print("  Val  :", X_val.shape[0])
    print("  Test :", X_test.shape[0])

    return X_train, X_val, X_test, y_train, y_val, y_test


# Train XGBoost Binary Classifier

class BoosterWrapper:
    """
    A lightweight wrapper that mimics XGBClassifier predict/predict_proba,
    but internally uses a trained Booster. This avoids sklearn attribute
    restrictions and works for all XGBoost versions.
    """
    def __init__(self, booster):
        self.booster = booster

    def predict_proba(self, X):
        dX = xgb.DMatrix(X)
        probs = self.booster.predict(dX)
        # Return 2-column soft probabilities like sklearn:
        # column 0 = P(class=0), column 1 = P(class=1)
        return np.column_stack([1 - probs, probs])

    def predict(self, X):
        probs = self.predict_proba(X)[:, 1]
        return (probs >= 0.5).astype(int)

    def save_model(self, path):
        self.booster.save_model(path)


def train_xgboost_binary(X_train, y_train, X_val, y_val):
    """
    Train binary XGBoost model using low-level Booster API with early stopping.
    Returns a BoosterWrapper that behaves like a classifier.
    """
    dtrain = xgb.DMatrix(X_train, label=y_train)
    dval   = xgb.DMatrix(X_val, label=y_val)

    params = {
        "objective": "binary:logistic",
        "eval_metric": "logloss",
        "eta": 0.03,
        "max_depth": 8,
        "subsample": 0.8,
        "colsample_bytree": 0.8,
        "tree_method": "hist"
    }

    evals = [(dtrain, "train"), (dval, "val")]

    booster = xgb.train(
        params=params,
        dtrain=dtrain,
        num_boost_round=1000,
        evals=evals,
        early_stopping_rounds=50,
        verbose_eval=False
    )

    print(f"Best iteration: {booster.best_iteration}")

    # Return wrapped booster with sklearn-like API
    return BoosterWrapper(booster)

# Model Evaluation

def evaluate_model(model, X_train, y_train, X_val, y_val, X_test, y_test):
    """
    Print classification reports and confusion matrices for train/val/test.
    """
    def evaluate_split(name, Xs, ys):
        y_pred = model.predict(Xs)
        print(f"\n===== {name} =====")
        print(classification_report(ys, y_pred, digits=4))
        print("Confusion matrix:")
        print(confusion_matrix(ys, y_pred))

    evaluate_split("TRAIN", X_train, y_train)
    evaluate_split("VAL", X_val, y_val)
    evaluate_split("TEST", X_test, y_test)


# Main pipeline

def main():
    # 1) Parse all CVAT XMLs → labels DataFrame
    df = build_labels_dataframe(WAVES)
    print("Total images in labels DF:", len(df))
    print(df['label'].value_counts(dropna=False))

    # IMPORTANT: You must ensure some images truly have label=0
    # (i.e., at least a few frames with no bounding boxes).
    # If label=1 for all rows, the model cannot do binary classification.

    # 2) Extract CNN features
    X, y = extract_features(df, cache_path=FEATURE_CACHE)
    print("Feature matrix shape:", X.shape)
    print("Labels shape:", y.shape)

    # 3) 70/10/20 split
    X_train, X_val, X_test, y_train, y_val, y_test = split_70_10_20(X, y)

    # --- SAVE SPLITS FOR LATER EVALUATION ---
    # Save file paths for test set
    test_indices = np.arange(len(y))[(len(y_train)+len(y_val)):]  # indices in original DF
    df_test = df.iloc[test_indices].copy()
    test_image_paths_file = r"C:\UVM\SEED25_johnFork\Machine_Learning\XGBoost\test_image_paths.txt"
    test_labels_file = r"C:\UVM\SEED25_johnFork\Machine_Learning\XGBoost\test_labels.txt"

    df_test['full_path'].to_csv(test_image_paths_file, index=False, header=False)
    df_test['label'].to_csv(test_labels_file, index=False, header=False)
    print(f"Saved test image paths to: {test_image_paths_file}")
    print(f"Saved test labels to: {test_labels_file}")

    # 4) Train XGBoost
    model = train_xgboost_binary(X_train, y_train, X_val, y_val)

    # 5) Evaluate
    evaluate_model(model, X_train, y_train, X_val, y_val, X_test, y_test)

    # 6) Save model
    model_output_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\XGBoost\xgb_model.json"
    model.save_model(model_output_path)

    print(f"\nSaved XGBoost model to: {model_output_path}")


if __name__ == "__main__":
    main()
