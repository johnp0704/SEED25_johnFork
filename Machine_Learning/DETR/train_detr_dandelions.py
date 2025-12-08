import os
import torch
from torch.utils.data import DataLoader
import math
from torch.optim.lr_scheduler import LambdaLR
from transformers import DetrImageProcessor, DetrForObjectDetection, DetrConfig
from tqdm import tqdm
from dandelion_detr_dataset import DandelionDataset
import torchvision.transforms as T

torch.backends.cudnn.benchmark = True

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Paths
train_img_dir = r"C:\Users\Samuel\OneDrive\Documents\GitHub\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\train\images"
val_img_dir = r"C:\Users\Samuel\OneDrive\Documents\GitHub\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
train_ann = r"C:\Users\Samuel\OneDrive\Documents\GitHub\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_train.json"
val_ann = r"C:\Users\Samuel\OneDrive\Documents\GitHub\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

# Label mapping
id2label = {0: "no_object", 1: "dandelion"}
label2id = {"no_object": 0, "dandelion": 1}

# Augmentations
augment = T.Compose([
    T.ColorJitter(
        brightness=0.3,
        contrast=0.3,
        saturation=0.3,
        hue=0.05
    ),
])

# Model + Processor
config = DetrConfig.from_pretrained("facebook/detr-resnet-50")
config.num_labels = 2
config.id2label = id2label
config.label2id = label2id

processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")

model = DetrForObjectDetection.from_pretrained(
    "facebook/detr-resnet-50",
    config=config,
    ignore_mismatched_sizes=True
)
model.to(device)

# Freeze backbone for warmup
for name, param in model.named_parameters():
    if "backbone" in name:
        param.requires_grad = False

frozen_backbone_epochs = 3

# Dataset and loader
train_dataset = DandelionDataset(train_img_dir, train_ann, processor, augment=augment)
val_dataset   = DandelionDataset(val_img_dir,   val_ann,  processor)

def collate_fn(batch):
    batch = [b for b in batch if b is not None]
    if len(batch) == 0:
        return None
    pixel_values = torch.stack([b["pixel_values"] for b in batch])
    labels = [b["labels"] for b in batch]
    return {"pixel_values": pixel_values, "labels": labels}

train_loader = DataLoader(train_dataset, batch_size=2, shuffle=True, collate_fn=collate_fn)
val_loader   = DataLoader(val_dataset,   batch_size=4, shuffle=False, collate_fn=collate_fn)

# Optimizer
head_lr = 3e-4
backbone_lr = 3e-5

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

# Scheduuler
num_epochs = 120
num_warmup_epochs = 5

def lr_lambda(current_epoch: int):
    if current_epoch < num_warmup_epochs:
        return float(current_epoch + 1) / float(max(1, num_warmup_epochs))
    progress = (current_epoch - num_warmup_epochs) / float(max(1, num_epochs - num_warmup_epochs))
    return 0.5 * (1.0 + math.cos(math.pi * progress))

scheduler = LambdaLR(optimizer, lr_lambda=lr_lambda)

# Training loop
best_val_loss = float("inf")
best_model_path = r"C:\Users\Samuel\OneDrive\Documents\GitHub\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_best_new_n"

gradient_accumulation_steps = 8

for epoch in range(num_epochs):

    # Unfreeze backbone
    if epoch == frozen_backbone_epochs:
        print("Unfreezing backbone...")
        for name, param in model.named_parameters():
            if "backbone" in name:
                param.requires_grad = True

    # Training
    model.train()
    total_loss = 0.0

    current_lrs = [group["lr"] for group in optimizer.param_groups]
    print(f"\nEpoch [{epoch+1}/{num_epochs}] LRs: heads={current_lrs[1]:.6f}, backbone={current_lrs[0]:.6f}")

    optimizer.zero_grad()

    for step, batch in enumerate(tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs} - Training")):
        if batch is None:
            continue

        pixel_values = batch["pixel_values"].to(device)
        labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

        outputs = model(pixel_values=pixel_values, labels=labels)
        loss = outputs.loss / gradient_accumulation_steps

        loss.backward()

        if (step + 1) % gradient_accumulation_steps == 0 or (step + 1) == len(train_loader):
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            optimizer.zero_grad()

        total_loss += loss.item() * gradient_accumulation_steps

    scheduler.step()

    avg_train_loss = total_loss / max(1, len(train_loader))
    print(f"Training Loss: {avg_train_loss:.4f}")

    # Validation
    model.eval()
    val_loss = 0.0

    with torch.no_grad():
        for batch in tqdm(val_loader, desc=f"Epoch {epoch+1}/{num_epochs} Validation"):
            if batch is None:
                continue

            pixel_values = batch["pixel_values"].to(device)
            labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

            outputs = model(pixel_values=pixel_values, labels=labels)
            val_loss += outputs.loss.item()

    avg_val_loss = val_loss / max(1, len(val_loader))
    print(f"Validation Loss: {avg_val_loss:.4f}")

    # Save best
    if avg_val_loss < best_val_loss:
        best_val_loss = avg_val_loss
        os.makedirs(best_model_path, exist_ok=True)
        model.save_pretrained(best_model_path)
        processor.save_pretrained(best_model_path)
        print(f"Best model saved: {best_val_loss:.4f}")

# Save final model
save_path = r"C:\Users\Samuel\OneDrive\Documents\GitHub\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_final_new_n"
os.makedirs(save_path, exist_ok=True)
model.save_pretrained(save_path)
processor.save_pretrained(save_path)

print("Model training complete")
print("Save directory:")
print(os.path.abspath(save_path))

# Checks if model detects anything
import random
from PIL import Image, ImageDraw

print("\nRunning quick DETR sanity test")

# Load processor + model from final save path
test_processor = DetrImageProcessor.from_pretrained(save_path)
test_model = DetrForObjectDetection.from_pretrained(save_path).to(device)
test_model.eval()

# Pick a random validation image
sample_img_info = random.choice(list(val_dataset.id2img.values()))
sample_path = os.path.join(val_img_dir, sample_img_info["file_name"])
sample_image = Image.open(sample_path).convert("RGB")

# Run inference
inputs = test_processor(images=sample_image, return_tensors="pt").to(device)

with torch.no_grad():
    outputs = test_model(**inputs)

# Post process outputs into boxes
results = test_processor.post_process_object_detection(
    outputs,
    target_sizes=torch.tensor([[sample_image.height, sample_image.width]]).to(device),
    threshold=0.9
)[0]

draw = ImageDraw.Draw(sample_image)

det_count = 0
for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
    score = score.cpu().item()
    if score < 0.9:
        continue

    det_count += 1
    box = box.cpu().numpy()
    x1, y1, x2, y2 = box
    draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
    draw.text((x1, y1), f"dandelion {score:.2f}", fill="red")

out_path = "detr_test_output.jpg"
sample_image.save(out_path)

print(f"Output saved to: {out_path}")

if det_count > 0:
    print(f"Detected {det_count} objects.")
else:
    print("Fail, No bounding boxes detected.")
