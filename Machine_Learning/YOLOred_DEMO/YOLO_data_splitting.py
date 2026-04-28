import os
import shutil
import random

# -----------------------------
# CONFIGURATION
# -----------------------------
# Base folder containing "Wave" folders (the source)
BASE_DIR = r"C:\UVM\SEED\SEED25\Images\Testing Annotated\YOLOredannotations"

# Output folder (your requested destination)
DATA_DIR = r"C:\UVM\SEED\SEED25\Images\Testing Annotated\YOLOredannotations\Data"

# Set random seed for reproducibility
random.seed(42)

# -----------------------------
# SETUP OUTPUT STRUCTURE
# -----------------------------
# Remove existing Data folder if it exists
if os.path.exists(DATA_DIR):
    shutil.rmtree(DATA_DIR)

# Create split subfolders
splits = ["train", "val", "test"]
for split in splits:
    os.makedirs(os.path.join(DATA_DIR, "images", split))
    os.makedirs(os.path.join(DATA_DIR, "labels", split))

print(f"✅ Created clean output folder at:\n{DATA_DIR}\n")

# -----------------------------
# COLLECT ALL IMAGE-LABEL PAIRS
# -----------------------------
all_pairs = []

for folder in os.listdir(BASE_DIR):
    folder_path = os.path.join(BASE_DIR, folder)
    if os.path.isdir(folder_path):
        images_path = os.path.join(folder_path, "images")
        labels_path = os.path.join(folder_path, "labels")

        if os.path.exists(images_path) and os.path.exists(labels_path):
            for img_file in os.listdir(images_path):
                if img_file.endswith(".png"):
                    img_src = os.path.join(images_path, img_file)
                    label_src = os.path.join(labels_path, img_file.replace(".png", ".txt"))
                    if os.path.exists(label_src):
                        all_pairs.append((img_src, label_src))
                    else:
                        print(f"⚠️ WARNING: Missing label for {img_file} in {folder}")

print(f"✅ Total image-label pairs collected: {len(all_pairs)}\n")

# -----------------------------
# SPLIT INTO TRAIN / VAL / TEST
# -----------------------------
random.shuffle(all_pairs)
num_images = len(all_pairs)

train_split = int(0.7 * num_images)
val_split = int(0.9 * num_images)

splits_dict = {
    "train": all_pairs[:train_split],
    "val": all_pairs[train_split:val_split],
    "test": all_pairs[val_split:]
}

# -----------------------------
# COPY FILES INTO SPLIT FOLDERS
# -----------------------------
for split, pairs in splits_dict.items():
    print(f"Copying {len(pairs)} samples to {split} split...")
    for img_src, label_src in pairs:
        img_name = os.path.basename(img_src)
        label_name = os.path.basename(label_src)

        shutil.copy(img_src, os.path.join(DATA_DIR, "images", split, img_name))
        shutil.copy(label_src, os.path.join(DATA_DIR, "labels", split, label_name))

print("\n✅ Dataset successfully copied and split into train/val/test.")
print(f"📁 Final Data folder created at:\n{DATA_DIR}")
