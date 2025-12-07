import os
import torch
from torch.utils.data import DataLoader
import math
from torch.optim.lr_scheduler import LambdaLR
from transformers import DetrImageProcessor, DetrForObjectDetection
from tqdm import tqdm
from dandelion_detr_dataset import DandelionDataset
import torchvision.transforms as T

torch.backends.cudnn.benchmark = True

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Paths
train_img_dir = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\train\images"
val_img_dir = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
train_ann = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_train.json"
val_ann = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

id2label = {0: "dandelion"}
label2id = {"dandelion": 0}

# Augmentations
augment = T.Compose([
    T.RandomResizedCrop(
        size=(800, 800),
        scale=(0.8, 1.0),
        ratio=(0.9, 1.1)
    ),
    T.ColorJitter(
        brightness=0.3,
        contrast=0.3,
        saturation=0.3,
        hue=0.05
    ),
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

frozen_backbone_epochs = 20

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
head_lr = 1e-4
backbone_lr = 1e-5

backbone_params = []
non_backbone_params = []
for name, param in model.named_parameters():
    if "backbone" in name:
        backbone_params.append(param)
    else:
        non_backbone_params.append(param)

optimizer = torch.optim.AdamW(
    [
        {"params": backbone_params, "lr": backbone_lr, "weight_decay": 1e-4},
        {"params": non_backbone_params, "lr": head_lr, "weight_decay": 1e-4},
    ]
)

# Warmup
num_epochs = 60
num_warmup_epochs = 1

def lr_lambda(current_epoch: int):
    # Warmup into cosine decay
    if current_epoch < num_warmup_epochs:
        # Linear scaling
        return float(current_epoch + 1) / float(max(1, num_warmup_epochs))
    # Cosine decay
    progress = (current_epoch - num_warmup_epochs) / float(max(1, num_epochs - num_warmup_epochs))
    return 0.5 * (1.0 + math.cos(math.pi * progress))

scheduler = LambdaLR(optimizer, lr_lambda=lr_lambda)

best_val_loss = float("inf")
best_model_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_best"

for epoch in range(num_epochs):
    model.train()
    total_loss = 0.0

    # Unfreeze
    if epoch == frozen_backbone_epochs:
        print("Unfreezing backbone...")
        for name, param in model.named_parameters():
            if "backbone" in name:
                param.requires_grad = True

    # Print current LR
    current_lrs = [group["lr"] for group in optimizer.param_groups]
    print(f"\nEpoch [{epoch+1}/{num_epochs}] - LRs: heads={current_lrs[1]:.6f}, backbone={current_lrs[0]:.6f}")

    for batch in tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs} - Training"):
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

    # Step scheduler after each epoch
    scheduler.step()

    avg_train_loss = total_loss / max(1, len(train_loader))
    print(f"Training Loss: {avg_train_loss:.4f}")

    # Validate
    model.eval()
    val_loss = 0.0

    with torch.no_grad():
        for batch in tqdm(val_loader, desc=f"Epoch {epoch+1}/{num_epochs} - Validation"):
            if batch is None:
                continue

            pixel_values = batch["pixel_values"].to(device)
            labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

            outputs = model(pixel_values=pixel_values, labels=labels)
            val_loss += outputs.loss.item()

    avg_val_loss = val_loss / max(1, len(val_loader))
    print(f"Validation Loss: {avg_val_loss:.4f}")

    # Save best model
    if avg_val_loss < best_val_loss:
        best_val_loss = avg_val_loss
        os.makedirs(best_model_path, exist_ok=True)
        model.save_pretrained(best_model_path)
        processor.save_pretrained(best_model_path)
        print(f"Best model saved: {best_val_loss:.4f}")

# Save model
save_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_final"
os.makedirs(save_path, exist_ok=True)
model.save_pretrained(save_path)
processor.save_pretrained(save_path)

print("Model training complete")

# Confirm final save location
print("Model and processor successfully saved.")
print("Save directory:")
print(os.path.abspath(save_path))

