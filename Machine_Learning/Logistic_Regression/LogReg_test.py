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
import pickle # To save the model

# =================Config=================
IMG_SIZE = (64, 64) # Concatenate images to 64x64 for computational simplicity.
OUTPUT_DIR = r"C:\UVM\SEED25_johnFork\Machine_Learning\Logistic_Regression" #where to save model

#logistic regression data path
LR_DATA_ROOT = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\LogRegannotations"

#define paths for the organized data files
LR_IMAGE_ROOT = os.path.join(LR_DATA_ROOT, "images")
LR_LABEL_ROOT = os.path.join(LR_DATA_ROOT, "labels")

os.makedirs(OUTPUT_DIR, exist_ok=True)


# =================Helper Functions=================
def load_lr_dataset(split):
    '''
    Reads pre-generated text files, loads the images listed in them,
    and returns features (X) and labels (y).
    '''
    
    #define paths to the organized text files
    image_paths_file = os.path.join(LR_IMAGE_ROOT, split, "image_paths.txt")
    label_paths_file = os.path.join(LR_LABEL_ROOT, split, "binary_labels.txt")

    #error handling courtesy of Gemini 3
    if not os.path.exists(image_paths_file) or not os.path.exists(label_paths_file):
        raise FileNotFoundError(f"Required LR data files for '{split}' not found. Please run the label creation script first.")

    #read image paths and label binaries
    with open(image_paths_file, 'r') as f:
        image_paths = [p.strip() for p in f.readlines()]
        
    with open(label_paths_file, 'r') as f:
        #convert labels to ints
        y = np.array([int(l.strip()) for l in f.readlines()])
        
    X = [] #feature vectors
    
    print(f"Loading {len(image_paths)} images for {split} set.")

    #image processing
    for img_path in image_paths:
        img = cv2.imread(img_path)
        if img is None:
            #error handling courtesy of Gemini 3
            print(f"Warning: Could not read image at {img_path}. Skipping.")
            continue
    
        img_resized = cv2.resize(img, IMG_SIZE)
        img_flat = img_resized.flatten()
        X.append(img_flat)
        
    return np.array(X), y

# =================Main=================
if __name__ == "__main__":
    #load data
    X_train, y_train = load_lr_dataset("train")
    X_test, y_test = load_lr_dataset("test")

    print(f"\nTraining Data Shape: {X_train.shape}")
    print(f"Testing Data Shape:  {X_test.shape}")

    #scale data
    print("\nScaling data...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test) 

    #train
    print("Training Logistic Regression...")
    model = LogisticRegression(max_iter=1000, solver='lbfgs', verbose=1) 
    model.fit(X_train_scaled, y_train)

    #save model
    with open(os.path.join(OUTPUT_DIR, "logreg_model.pkl"), "wb") as f:
        pickle.dump(model, f)
    with open(os.path.join(OUTPUT_DIR, "scaler.pkl"), "wb") as f:
        pickle.dump(scaler, f)
    print(f"Model and scaler saved to {OUTPUT_DIR}")

    #evaluate model
    print("\nEvaluating...")
    y_pred = model.predict(X_test_scaled)
    
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred, target_names=["No Dandelion", "Dandelion"]))

    #confusion matrix
    cm = confusion_matrix(y_test, y_pred)
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=['No Dandelion', 'Dandelion'])
    disp.plot(cmap=plt.cm.Greens)
    plt.title('Logistic Regression Confusion Matrix')
    plt.show()