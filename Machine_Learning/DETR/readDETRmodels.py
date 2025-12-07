from safetensors.torch import load_file
import torch

# Path to your weights
weight_path = r"C:\UVM\SEED25_johnFork\Machine_Learning\DETR\detr-dandelions-model_best\model.safetensors"

# Load tensors into memory
tensors = load_file(weight_path)

print(f"{'Layer Name':<50} | {'Mean':<10} | {'Std':<10} | {'Min':<10} | {'Max':<10}")
print("-" * 100)

for name, tensor in tensors.items():
    # We focus on weights, not biases or internal metadata
    if "weight" in name:
        mean = tensor.mean().item()
        std = tensor.std().item()
        t_min = tensor.min().item()
        t_max = tensor.max().item()
        print(f"{name[:50]:<50} | {mean:10.4f} | {std:10.4f} | {t_min:10.4f} | {t_max:10.4f}")