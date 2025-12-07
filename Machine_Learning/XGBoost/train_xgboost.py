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

# Config

# Wave
WAVES = [
    {
        "name": "wave1",
        "xml_path": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 1\annotations.xml",
        "image_dir": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 1"
    },
    # {
    #     "name": "wave2",
    #     "xml_path": r"C:\path\to\wave2\annotations.xml",
    #     "image_dir": r"C:\path\to\wave2\images"
    # },
    {
        "name": "wave3",
        "xml_path": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 3\annotations.xml",
        "image_dir": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 3"
    },
    {
        "name": "wave4",
        "xml_path": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 4\annotations.xml",
        "image_dir": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 4"
    },
    {
        "name": "wave5",
        "xml_path": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 5\annotations.xml",
        "image_dir": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 5"
    },
    {
        "name": "wave6",
        "xml_path": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 6\annotations.xml",
        "image_dir": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 6"
    },
    # {
    #     "name": "wave7",
    #     "xml_path": r"C:\path\to\wave1\annotations.xml",
    #     "image_dir": r"C:\path\to\wave1\images"
    # },
    {
        "name": "wave8",
        "xml_path": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\XGBoostAnnotations\Wave 8\annotations.xml",
        "image_dir": r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 8"
    }
]

IMG_SIZE = (224, 224) # ResNet50 input size
RANDOM_STATE = 42
FEATURE_CACHE = None


# Get image labels from CVAT XMLs
def parse_cvat_xml(xml_path, image_dir, wave_name):
    # Parses CVAT XML and returns dataframe with image paths and labels
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
    # Combines waves into dataframe
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
    if not os.path.exists(img_path):
        raise FileNotFoundError(f"Image not found: {img_path}")

    img = load_img(img_path, target_size=IMG_SIZE)
    x = img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)
    feat = resnet_model.predict(x, verbose=0)[0]  # shape (2048,)
    return feat


def extract_features(df, cache_path=None):
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
            print(f"Processed {idx + 1}/{len(df)} images")

    X = np.array(X_list)
    y = np.array(y_list)

    if cache_path is not None:
        np.save(cache_path, {"X": X, "y": y})
        print(f"Saved features to cache: {cache_path}")

    return X, y


# 70/20/10 Split

def split_70_20_10(X, y, random_state=RANDOM_STATE):
    X_train, X_temp, y_train, y_temp = train_test_split(
        X, y,
        test_size=0.30,
        random_state=random_state,
        stratify=y
    )

    # 20/10 split from remaining 30% → val = 2/3 of 30% = 20%, test = 1/3 = 10%
    X_val, X_test, y_val, y_test = train_test_split(
        X_temp, y_temp,
        test_size=1/3,
        random_state=random_state,
        stratify=y_temp
    )

    print("Split sizes:")
    print("  Train:", X_train.shape[0])
    print("  Val  :", X_val.shape[0])
    print("  Test :", X_test.shape[0])

    return X_train, X_val, X_test, y_train, y_val, y_test


# Train XGBoost Binary Classifier

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

    def save_model(self, path):
        self.booster.save_model(path)


def train_xgboost_binary(X_train, y_train, X_val, y_val):
    # Train XGBoost binary classifier
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
    # Print classification report and confusion matrix for each split
    def evaluate_split(name, Xs, ys):
        y_pred = model.predict(Xs)
        print(f"\n{name}")
        print(classification_report(ys, y_pred, digits=4))
        print("Confusion matrix:")
        print(confusion_matrix(ys, y_pred))

    evaluate_split("TRAIN", X_train, y_train)
    evaluate_split("VAL", X_val, y_val)
    evaluate_split("TEST", X_test, y_test)


# Main pipeline
def main():
    df = build_labels_dataframe(WAVES)
    print("Total images in labels DF:", len(df))
    print(df['label'].value_counts(dropna=False))

    # Extract CNN features
    X, y = extract_features(df, cache_path=FEATURE_CACHE)
    print("Feature matrix shape:", X.shape)
    print("Labels shape:", y.shape)

    # 70/20/10 split
    X_train, X_val, X_test, y_train, y_val, y_test = split_70_20_10(X, y)

    # Train XGBoost
    model = train_xgboost_binary(X_train, y_train, X_val, y_val)

    # Evaluate
    evaluate_model(model, X_train, y_train, X_val, y_val, X_test, y_test)

    # Save model
    model_output_path = r"C:\Users\samst\Downloads\xgboost_model.json"
    model.save_model(model_output_path)
    print(f"\nSaved XGBoost model to: {model_output_path}")


if __name__ == "__main__":
    main()
