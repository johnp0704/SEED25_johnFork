import os
import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Load model
model_path = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Machine_Learning\DETR\detr-dandelion"

processor = DetrImageProcessor.from_pretrained(model_path)
model = DetrForObjectDetection.from_pretrained(model_path)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)
model.eval()

# Single image
def analyze_image(image_path, score_threshold=0.5):
    image = Image.open(image_path).convert("RGB")

    inputs = processor(images=image, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_object_detection(
        outputs,
        target_sizes=[image.size[::-1]],
        threshold=score_threshold
    )[0]

    return image, results

# Visulaize and Save
def draw_and_save(image, results, save_path):
    fig, ax = plt.subplots(1)
    ax.imshow(image)
    ax.axis("off")

    for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
        x1, y1, x2, y2 = box.tolist()

        rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                 linewidth=2, edgecolor='red', facecolor='none')
        ax.add_patch(rect)

        cls = model.config.id2label[label.item()]
        ax.text(x1, y1, f"{cls}: {score:.2f}", color="yellow", fontsize=10)

    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)

# Run DETR on all images in a folder
def run_folder(input_folder, output_folder, threshold=0.5):

    os.makedirs(output_folder, exist_ok=True)

    image_files = [
        os.path.join(input_folder, f)
        for f in os.listdir(input_folder)
        if f.lower().endswith((".jpg", ".jpeg", ".png"))
    ]

    print(f"Found {len(image_files)} images.")
    print("Running inference...")

    for i, img_path in enumerate(image_files, 1):
        print(f"[{i}/{len(image_files)}] Processing: {os.path.basename(img_path)}")

        image, results = analyze_image(img_path, score_threshold=threshold)

        save_name = os.path.splitext(os.path.basename(img_path))[0] + "_detr.png"
        save_path = os.path.join(output_folder, save_name)

        draw_and_save(image, results, save_path)

    print("\nResults saved to:", output_folder)

if __name__ == "__main__":
    input_folder = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Preliminary Images\Dandelion\RGB\Wave_3_test\images\train"
    output_folder = r"C:\Users\samst\OneDrive\Documents\GitHub\Micro Final Project\SEED25_johnFork\Images\Testing Annotated\DETRannotations\test_images_annotated"

    run_folder(input_folder, output_folder, threshold=0.6)
