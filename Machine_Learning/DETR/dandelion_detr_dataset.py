import os
import torch
from torch.utils.data import Dataset
from PIL import Image
import json

class DandelionDataset(Dataset):
    def init(self, images_dir, annotation_file, processor, augment=None):
        self.images_dir = images_dir
        self.processor = processor
        self.augment = augment

        with open(annotation_file, 'r') as f:
            coco = json.load(f)

        self.id2img = {img['id']: img for img in coco['images']}

        self.img2anns = {img_id: [] for img_id in self.id2img.keys()}
        for ann in coco['annotations']:
            if ann['image_id'] in self.img2anns:
                self.img2anns[ann['image_id']].append(ann)

        self.ids = list(self.id2img.keys())

    def len(self):
        return len(self.ids)

    def getitem(self, idx):
        img_id = self.ids[idx]
        img_info = self.id2img[img_id]
        file_name = img_info['file_name']

        path = os.path.join(self.images_dir, file_name)
        image = Image.open(path).convert('RGB')

        anns = self.img2anns[img_id]

        annotations = []
        for a in anns:
            x, y, w, h = a['bbox']

            annotations.append({
                'bbox': [x, y, w, h],
                'category_id': a['category_id'],
                'area': w * h,
                'iscrowd': 0
            })

        target = {'image_id': img_id, 'annotations': annotations}

        if self.augment is not None:
            image = self.augment(image)

        encoding = self.processor(
            images=image,
            annotations=target,
            return_tensors='pt'
        )

        pixel_values = encoding['pixel_values'].squeeze()
        labels = encoding['labels'][0]

        return {'pixel_values': pixel_values, 'labels': labels}