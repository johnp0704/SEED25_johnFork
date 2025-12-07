import os
import json
import random
import shutil

# Coco annotations
COCO_WAVE_DIRS = [
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\COCO Wave 1",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\COCO Wave 3",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\COCO Wave 4",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\COCO Wave 5",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\COCO Wave 6",
]

# Imagaes
RAW_WAVE_DIRS = [
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 1",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 3",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 4",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 5",
    r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave 6",
]

# DETR output
DATASET_ROOT = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr"

TRAIN_RATIO = 0.7
VAL_RATIO   = 0.1
TEST_RATIO  = 0.2


# Search for raw image in wave dirs
def find_raw_image(filename):
    for wave in RAW_WAVE_DIRS:
        path = os.path.join(wave, filename)
        if os.path.exists(path):
            return path
    return None


def main():

    merged_images = []
    merged_annotations = []
    categories_ref = None

    next_image_id = 1
    next_ann_id = 1

    # Merge COCO annotations
    for wave_dir in COCO_WAVE_DIRS:
        ann_path = os.path.join(wave_dir, "annotations", "instances_default.json")

        print(f"Loading {ann_path}")
        with open(ann_path, "r") as f:
            coco = json.load(f)

        if categories_ref is None:
            categories_ref = coco["categories"]

        # Remap image IDs
        old_to_new = {}

        for img in coco["images"]:
            old_id = img["id"]
            img["id"] = next_image_id
            old_to_new[old_id] = next_image_id
            merged_images.append(img)
            next_image_id += 1

        # Remap annotations
        for ann in coco["annotations"]:
            ann["id"] = next_ann_id
            ann["image_id"] = old_to_new[ann["image_id"]]
            merged_annotations.append(ann)
            next_ann_id += 1

    # Random split 70/20/10
    random.seed(42)
    random.shuffle(merged_images)

    total = len(merged_images)
    n_train = int(total * TRAIN_RATIO)
    n_val   = int(total * VAL_RATIO)
    n_test  = total - n_train - n_val

    train_imgs = merged_images[:n_train]
    val_imgs   = merged_images[n_train:n_train+n_val]
    test_imgs  = merged_images[n_train+n_val:]

    train_ids = {img["id"] for img in train_imgs}
    val_ids   = {img["id"] for img in val_imgs}
    test_ids  = {img["id"] for img in test_imgs}

    train_anns = [a for a in merged_annotations if a["image_id"] in train_ids]
    val_anns   = [a for a in merged_annotations if a["image_id"] in val_ids]
    test_anns  = [a for a in merged_annotations if a["image_id"] in test_ids]

    # Create output folder
    train_dir = os.path.join(DATASET_ROOT, "train", "images")
    val_dir   = os.path.join(DATASET_ROOT, "val", "images")
    test_dir  = os.path.join(DATASET_ROOT, "test", "images")
    ann_dir   = os.path.join(DATASET_ROOT, "annotations")

    for d in [train_dir, val_dir, test_dir, ann_dir]:
        os.makedirs(d, exist_ok=True)

    # Copy images
    def copy_set(img_list, dest):
        for img in img_list:
            fname = img["file_name"]
            src = find_raw_image(fname)
            if not src:
                print(f"Missing raw image: {fname}")
                continue
            shutil.copy2(src, os.path.join(dest, fname))

    print("Copying training images...")
    copy_set(train_imgs, train_dir)

    print("Copying validation images...")
    copy_set(val_imgs, val_dir)

    print("Copying test images...")
    copy_set(test_imgs, test_dir)

    # Write new json
    def save_json(name, imgs, anns):
        out = {"images": imgs, "annotations": anns, "categories": categories_ref}
        with open(os.path.join(ann_dir, name), "w") as f:
            json.dump(out, f)

    save_json("instances_train.json", train_imgs, train_anns)
    save_json("instances_val.json", val_imgs, val_anns)
    save_json("instances_test.json", test_imgs, test_anns)

    print("\nDONE! Merged + Split COCO Dataset Created")
    print(f"Total images: {total}")
    print(f"Train: {len(train_imgs)}")
    print(f"Val:   {len(val_imgs)}")
    print(f"Test:  {len(test_imgs)}")


if __name__ == "__main__":
    main()
