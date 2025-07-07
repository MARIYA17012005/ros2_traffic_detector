import os
from pathlib import Path

# Paths
image_dir = Path.home() / "ros2_ws/datasets/rgb/additional"
label_dir = image_dir / "labels"
label_dir.mkdir(exist_ok=True)

# Generate dummy labels
for img_file in image_dir.glob("*.png"):
    txt_file = label_dir / (img_file.stem + ".txt")
    
    # Dummy box in YOLO format
    dummy_label = "0 0.5 0.5 0.3 0.3\n"

    with open(txt_file, "w") as f:
        f.write(dummy_label)

print(f"✅ Dummy YOLO labels created in: {label_dir}")
