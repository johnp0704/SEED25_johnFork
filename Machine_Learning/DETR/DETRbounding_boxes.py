# DETR bounding boxes
import os
import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt

# Paths
MODEL_PATH = r"C:\UVM\SEED\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_best"
IMAGE_DIR  = r"C:\UVM\SEED\SEED25_johnFork\Images\Testing Annotated\DETRannotations\dandelion_dataset_detr\test\images"
OUTPUT_DIR = r"C:\UVM\SEED\SEED25_johnFork\Machine_Learning\DETR\Vis_results"
THRESHOLD  = 0.9  # DETR usually needs very low threshold


# Load model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

processor = DetrImageProcessor.from_pretrained(MODEL_PATH)
model = DetrForObjectDetection.from_pretrained(MODEL_PATH).to(device)
model.eval()

os.makedirs(OUTPUT_DIR, exist_ok=True)


def visualize_image(image_path):
    image = Image.open(image_path).convert("RGB")
    width, height = image.size

    # Prepare input
    inputs = processor(images=image, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    # DETR post processing
    results = processor.post_process_object_detection(
        outputs,
        target_sizes=[(height, width)],
        threshold=THRESHOLD
    )[0]

    scores = results["scores"]
    labels = results["labels"]
    boxes  = results["boxes"]

    print(f"\nImage: {image_path}")
    print("Detections:", len(boxes))

    draw = ImageDraw.Draw(image)

    # Font
    try:
        font = ImageFont.truetype("arial.ttf", 20)
    except:
        font = ImageFont.load_default()

    # Draw detections
    for box, score, label in zip(boxes, scores, labels):
        x1, y1, x2, y2 = box.tolist()
        draw.rectangle([x1, y1, x2, y2], outline="red", width=3)

        caption = f"dandelion {score:.2f}"
        draw.text((x1, y1 - 10), caption, fill="red", font=font)

    # Save + show
    save_path = os.path.join(OUTPUT_DIR, os.path.basename(image_path))
    image.save(save_path)

    plt.figure(figsize=(8, 8))
    plt.imshow(image)
    plt.title(os.path.basename(image_path))
    plt.axis("off")
    plt.show()


# Run on images
if __name__ == "__main__":
    all_images = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(("jpg", "png", "jpeg"))]

    for img_file in all_images:
        visualize_image(os.path.join(IMAGE_DIR, img_file))