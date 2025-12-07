import os
import torch
from torch.utils.data import DataLoader
from torch.optim.lr_scheduler import StepLR
from transformers import DetrImageProcessor, DetrForObjectDetection
from tqdm import tqdm
from dandelion_detr_dataset import DandelionDataset
import torchvision.transforms as T

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Paths
train_img_dir = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\train\images"
val_img_dir   = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
train_ann     = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_train.json"
val_ann       = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

id2label = {0: "dandelion"}
label2id = {"dandelion": 0}

# Augmentations
augment = T.Compose([
    T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.3),
    T.RandomHorizontalFlip(p=0.5),
    T.RandomVerticalFlip(p=0.2),
    T.RandomRotation(10),
])

# Model
processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")

model = DetrForObjectDetection.from_pretrained(
    "facebook/detr-resnet-50",
    ignore_mismatched_sizes=True,
    num_labels=1,
    id2label=id2label,
    label2id=label2id
)

model.to(device)

# Freeze backbone
for name, param in model.named_parameters():
    if "backbone" in name:
        param.requires_grad = False

frozen_backbone_epochs = 10

# Dataset
train_dataset = DandelionDataset(train_img_dir, train_ann, processor, augment=augment)
val_dataset   = DandelionDataset(val_img_dir,   val_ann,  processor)

def collate_fn(batch):
    batch = [b for b in batch if b is not None and len(b["labels"]["class_labels"]) > 0]
    if len(batch) == 0:
        return None
    pixel_values = torch.stack([b["pixel_values"] for b in batch])
    labels = [b["labels"] for b in batch]
    return {"pixel_values": pixel_values, "labels": labels}

train_loader = DataLoader(train_dataset, batch_size=4, shuffle=True, collate_fn=collate_fn)
val_loader   = DataLoader(val_dataset,   batch_size=4, shuffle=False, collate_fn=collate_fn)

# Optimizer
optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-4)
scheduler = StepLR(optimizer, step_size=15, gamma=0.5)

# Train loop
num_epochs = 40

for epoch in range(num_epochs):
    model.train()
    total_loss = 0

    # Unfreeze
    if epoch == frozen_backbone_epochs:
        print("Unfreezing backbone...")
        for name, param in model.named_parameters():
            if "backbone" in name:
                param.requires_grad = True

    for batch in tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs}"):
        if batch is None:
            continue

        pixel_values = batch["pixel_values"].to(device)
        labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

        outputs = model(pixel_values=pixel_values, labels=labels)
        loss = outputs.loss

        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()

        total_loss += loss.item()

    scheduler.step()

    print(f"Training Loss: {total_loss / len(train_loader):.4f}")

    # Validate
    model.eval()
    val_loss = 0

    with torch.no_grad():
        for batch in val_loader:
            if batch is None:
                continue
            pixel_values = batch["pixel_values"].to(device)
            labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

            outputs = model(pixel_values=pixel_values, labels=labels)
            val_loss += outputs.loss.item()

    print(f"Validation Loss: {val_loss / len(val_loader):.4f}")


# Save model
save_path = r"C:\Users\samst\Downloads\detr-dandelions-model_final"
os.makedirs(save_path, exist_ok=True)
model.save_pretrained(save_path)
processor.save_pretrained(save_path)

print("Model training complete")
