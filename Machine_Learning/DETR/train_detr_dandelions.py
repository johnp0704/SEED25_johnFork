import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="torch.nn.modules.module")
import os
import torch
from torch.utils.data import DataLoader
import math
from torch.optim.lr_scheduler import LambdaLR
from transformers import DetrImageProcessor, DetrForObjectDetection
from tqdm import tqdm
from dandelion_detr_dataset import DandelionDataset
import torchvision.transforms as T
from torchmetrics.detection.mean_ap import MeanAveragePrecision

def collate_fn(batch):
        batch = [b for b in batch if b is not None]
        pixel_values = torch.stack([b["pixel_values"] for b in batch])
        labels = [b["labels"] for b in batch]
        orig_sizes = torch.stack([b["orig_size"] for b in batch])
        return {"pixel_values": pixel_values, "labels": labels, "orig_sizes": orig_sizes}

def main():
    print("Main thread started. Spawning workers...")
    torch.backends.cudnn.benchmark = True
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Paths
    train_img_dir = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\train\images"
    val_img_dir = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\val\images"
    train_ann = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_train.json"
    val_ann = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\annotations\instances_val.json"

    # 2. INITIALIZE METRIC BEFORE THE LOOP
    metric = MeanAveragePrecision(iou_type="bbox")
    best_val_map = 0.0  # track best map

    id2label = {0: "dandelion"}
    label2id = {"dandelion": 0}

    # Augmentations (Keep disabled or minimal for initial convergence)
    augment = T.Compose([
        T.RandomHorizontalFlip(p=0.5), # Standard for vegetation datasets
        # T.RandomResizedCrop(size=(800, 800), scale=(0.8, 1.0)), # Re-enable later
    ])

    # Model
    processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
    model = DetrForObjectDetection.from_pretrained(
        "facebook/detr-resnet-50", #"facebook/detr-resnet-50" if starting from scratch again
        ignore_mismatched_sizes=True,
        num_labels=1,
        id2label=id2label,
        label2id=label2id
    )

    model.to(device)

    # Freeze backbone initially
    for name, param in model.named_parameters():
        if "backbone" in name:
            param.requires_grad = False

    frozen_backbone_epochs = 20

    # Dataset
    train_dataset = DandelionDataset(train_img_dir, train_ann, processor, augment=augment)
    val_dataset   = DandelionDataset(val_img_dir,   val_ann,   processor)

    

    # DATA LOADER OPTIMIZATION
    train_loader = DataLoader(
        train_dataset, 
        batch_size=8, 
        shuffle=True, 
        collate_fn=collate_fn,
        num_workers=6,   # Multi-core processing
        pin_memory=True,  # Fast CPU-to-GPU copy
        persistent_workers=False
    )

    val_loader = DataLoader(
        val_dataset, 
        batch_size=8, 
        shuffle=False, 
        collate_fn=collate_fn,
        num_workers=6,
        pin_memory=True,
        persistent_workers=False
    )

    # Optimizer
    head_lr = 1e-4
    backbone_lr = 1e-5
    backbone_params = [p for n, p in model.named_parameters() if "backbone" in n]
    non_backbone_params = [p for n, p in model.named_parameters() if "backbone" not in n]

    optimizer = torch.optim.AdamW([
            {"params": backbone_params, "lr": backbone_lr, "weight_decay": 1e-4},
            {"params": non_backbone_params, "lr": head_lr, "weight_decay": 1e-4},
        ])

    num_epochs = 60
    scheduler = LambdaLR(optimizer, lr_lambda=lambda e: 0.5 * (1.0 + math.cos(math.pi * e / num_epochs)))

    # MIXED PRECISION INITIALIZATION
    scaler = torch.amp.GradScaler()
    best_model_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_best"

    for epoch in range(num_epochs):
        if epoch == frozen_backbone_epochs:
            print("Unfreezing backbone...")
            for name, param in model.named_parameters():
                if "backbone" in name: param.requires_grad = True

        model.train()
        total_loss = 0.0

        for batch in tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs} - Training"):
            if batch is None: continue

            pixel_values = batch["pixel_values"].to(device)
            labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]

            # FORWARD PASS WITH AUTO-MIXED PRECISION
            with torch.amp.autocast(device_type='cuda', dtype=torch.float16):
                outputs = model(pixel_values=pixel_values, labels=labels)
                loss = outputs.loss

            optimizer.zero_grad()
            
            # SCALER STEPS FOR MIXED PRECISION
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            scaler.step(optimizer)
            scaler.update()

            total_loss += loss.item()

        scheduler.step()
        print(f"Training Loss: {total_loss / len(train_loader):.4f}")

        # VALIDATION PHASE
        model.eval()
        val_loss = 0.0
        metric.reset()

        with torch.no_grad():
            for batch in tqdm(val_loader, desc=f"Epoch {epoch+1} - Validation"):
                if batch is None: continue
                pixel_values = batch["pixel_values"].to(device)
                labels = [{k: v.to(device) for k, v in t.items()} for t in batch["labels"]]
                orig_sizes = batch["orig_sizes"].to(device)

                outputs = model(pixel_values=pixel_values, labels=labels)
                val_loss += outputs.loss.item()

                results = processor.post_process_object_detection(outputs, target_sizes=orig_sizes, threshold=0.01)
                preds = [dict(boxes=res["boxes"], scores=res["scores"], labels=res["labels"]) for res in results]
                
                targets = []
                for i, l in enumerate(labels):
                    h, w = orig_sizes[i]
                    raw_boxes = l["boxes"] * torch.tensor([w, h, w, h], device=device)
                    cx, cy, bw, bh = raw_boxes.unbind(-1)
                    new_boxes = torch.stack([cx - 0.5 * bw, cy - 0.5 * bh, cx + 0.5 * bw, cy + 0.5 * bh], dim=-1)
                    targets.append(dict(boxes=new_boxes, labels=l["class_labels"]))
                
                metric.update(preds, targets)

        mAP_results = metric.compute()
        current_map = mAP_results["map"].item()
        print(f"Validation Loss: {val_loss / len(val_loader):.4f} | mAP: {current_map:.4f}")

        if current_map > best_val_map:
            best_val_map = current_map
            os.makedirs(best_model_path, exist_ok=True)
            model.save_pretrained(best_model_path)
            processor.save_pretrained(best_model_path)
            print(f"New best model saved with mAP: {current_map:.4f}")

    # FINAL SAVE
    save_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_final"
    os.makedirs(save_path, exist_ok=True)
    model.save_pretrained(save_path)
    processor.save_pretrained(save_path)
    print("Training complete and final model saved.")

# ENTRY POINT FOR WINDOWS MULTIPROCESSING
if __name__ == "__main__":
    main()