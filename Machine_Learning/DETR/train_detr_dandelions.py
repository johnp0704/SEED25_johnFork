import os
from torch.utils.data import DataLoader
import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from tqdm import tqdm
from dandelion_detr_dataset import DandelionDataset

# Device setup
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Paths
train_img_dir = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\train\images"
val_img_dir   = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
train_ann     = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_train.json"
val_ann       = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

# Labels
id2label = {0: "dandelion"}
label2id = {"dandelion": 0}

# Model and processor
processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
model = DetrForObjectDetection.from_pretrained(
    "facebook/detr-resnet-50",
    ignore_mismatched_sizes=True,
    num_labels=len(id2label),
    id2label=id2label,
    label2id=label2id,
)
model.to(device)

# Datasets
train_dataset = DandelionDataset(train_img_dir, train_ann, processor)
val_dataset   = DandelionDataset(val_img_dir, val_ann, processor)

print("Total train images:", len(train_dataset))
for i in range(3):
    s = train_dataset[i]
    print(f"Sample {i} -> boxes:", s["labels"]["boxes"].shape)

# collate
def collate_fn(batch):
    # Drop empty samples
    batch = [
        b for b in batch
        if b is not None and "labels" in b
        and len(b["labels"]["class_labels"]) > 0
    ]
    if len(batch) == 0:
        return None

    pixel_values = torch.stack([b["pixel_values"] for b in batch])
    labels = [b["labels"] for b in batch]
    return {"pixel_values": pixel_values, "labels": labels}

# Load data
train_loader = DataLoader(train_dataset, batch_size=4, shuffle=True, collate_fn=collate_fn)
val_loader   = DataLoader(val_dataset, batch_size=4, shuffle=False, collate_fn=collate_fn)

# Optimizer
optimizer = torch.optim.AdamW(model.parameters(), lr=5e-5)
num_epochs = 10

# Training loop
for epoch in range(num_epochs):
    model.train()
    total_loss = 0.0

    for batch in tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs}"):
        if batch is None:
            continue  # skip empty batch

        pixel_values = batch["pixel_values"].to(device)
        labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

        outputs = model(pixel_values=pixel_values, labels=labels)
        loss = outputs.loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total_loss += loss.item()

    avg_loss = total_loss / len(train_loader)
    print(f"Epoch {epoch+1} training loss: {avg_loss:.4f}")

    # Validate
    model.eval()
    val_loss = 0.0
    with torch.no_grad():
        for batch in val_loader:
            if batch is None:
                continue
            pixel_values = batch["pixel_values"].to(device)
            labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]
            outputs = model(pixel_values=pixel_values, labels=labels)
            val_loss += outputs.loss.item()
    val_loss /= len(val_loader)
    print(f"Epoch {epoch+1} validation loss: {val_loss:.4f}")

# Save model
save_path = r"C:\Users\samst\Downloads\detr-dandelions-model"

os.makedirs(save_path, exist_ok=True)

model.save_pretrained(save_path)
processor.save_pretrained(save_path)

# --- Verification section ---
required_files = [
    "config.json",
    "preprocessor_config.json",
    "pytorch_model.bin"
]

print("\nVerifying saved model...")

if not os.path.exists(save_path):
    raise FileNotFoundError(f"❌ Save directory does NOT exist: {save_path}")

missing = []
for f in required_files:
    fp = os.path.join(save_path, f)
    if not os.path.exists(fp):
        missing.append(f)

if len(missing) == 0:
    print("✅ Model saved successfully!")
    print("📁 Saved to:", save_path)
    print("📦 Files found:")
    for f in required_files:
        print("   ✔", f)
else:
    print("❌ Missing model files:")
    for f in missing:
        print("   ✘", f)
    raise RuntimeError("Model save failed — required files missing.")

