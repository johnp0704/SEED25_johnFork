import os
import json
import random
import shutil

coco_ann_file = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\annotations\instances_default.json"
cvat_images_dir = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave_3_test\images\train"

dataset_root = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr"
train_images_dir = os.path.join(dataset_root, "train", "images")
val_images_dir   = os.path.join(dataset_root, "val", "images")
ann_output_dir   = os.path.join(dataset_root, "annotations")

train_ratio = 0.8

def main():
    os.makedirs(train_images_dir, exist_ok=True)
    os.makedirs(val_images_dir, exist_ok=True)
    os.makedirs(ann_output_dir, exist_ok=True)

    with open(coco_ann_file, "r") as f:
        coco = json.load(f)

    images = coco["images"]
    annotations = coco["annotations"]
    categories = coco["categories"]

    random.seed(42)
    random.shuffle(images)

    n_train = int(len(images) * train_ratio)
    train_images = images[:n_train]
    val_images = images[n_train:]

    train_ids = set(img["id"] for img in train_images)
    val_ids   = set(img["id"] for img in val_images)

    train_annotations = [ann for ann in annotations if ann["image_id"] in train_ids]
    val_annotations   = [ann for ann in annotations if ann["image_id"] in val_ids]

    def copy_images(image_list, dest_dir):
        for img_info in image_list:
            src_path = os.path.join(cvat_images_dir, img_info["file_name"])
            dst_path = os.path.join(dest_dir, img_info["file_name"])
            if os.path.exists(src_path):
                shutil.copy2(src_path, dst_path)
            else:
                print(f"Warning: {src_path} not found")

    print("Copying training images...")
    copy_images(train_images, train_images_dir)

    print("Copying validation images...")
    copy_images(val_images, val_images_dir)

    train_coco = {"images": train_images, "annotations": train_annotations, "categories": categories}
    val_coco   = {"images": val_images, "annotations": val_annotations, "categories": categories}

    with open(os.path.join(ann_output_dir, "instances_train.json"), "w") as f:
        json.dump(train_coco, f)
    with open(os.path.join(ann_output_dir, "instances_val.json"), "w") as f:
        json.dump(val_coco, f)

    print(f"Created dataset in: {dataset_root}")
    print(f"Train: {len(train_images)} images")
    print(f"Val:   {len(val_images)} images")

if __name__ == "__main__":
    main()
