import os
import torch
from torch.utils.data import Dataset
from PIL import Image
import json

class DandelionDataset(Dataset):
    def __init__(self, images_dir, annotation_file, processor):
        """
        Args:
            images_dir (str): path to images folder
            annotation_file (str): COCO-style JSON
            processor (DetrImageProcessor)
        """
        self.images_dir = images_dir
        self.processor = processor

        with open(annotation_file, "r") as f:
            coco = json.load(f)

        # Map image to file name
        self.id2img = {img["id"]: img for img in coco["images"]}

        # Groups annotations per image
        self.img2anns = {img_id: [] for img_id in self.id2img.keys()}
        for ann in coco["annotations"]:
            if ann["image_id"] in self.img2anns:
                self.img2anns[ann["image_id"]].append(ann)
        self.ids = list(self.id2img.keys())

    def __len__(self):
        return len(self.ids)

    def __getitem__(self, idx):
        img_id = self.ids[idx]
        img_info = self.id2img[img_id]
        file_name = img_info["file_name"]
        path = os.path.join(self.images_dir, file_name)
        image = Image.open(path).convert("RGB")

        anns = self.img2anns[img_id]
        annotations = []
        for a in anns:
            x, y, w, h = a["bbox"]
            area = w * h
            annotations.append({
                "bbox": [x, y, w, h],
                "category_id": a["category_id"],
                "area": area,
                "iscrowd": 0
            })

        # Creates valid COCO annotation
        target = {"image_id": img_id, "annotations": annotations}

        # If no boxes, skips image
        if len(annotations) == 0:
            # skip forward until you find a non-empty image
            return self.__getitem__((idx + 1) % len(self))

        # preprocess
        encoding = self.processor(images=image, annotations=target, return_tensors="pt")
        pixel_values = encoding["pixel_values"].squeeze()
        labels = encoding["labels"][0]

        return {"pixel_values": pixel_values, "labels": labels}
    
        

