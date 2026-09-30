"""
idd_lite_smoke_test.py
======================
IDD Lite smoke test for SIH PS-26037 submission.

Parses the pixel-wise semantic segmentation label maps from IDD Lite
(idd20k_lite), tallies class-pixel distributions for ODD-relevant classes,
and produces one composite sample image with segmentation overlay.

IDD label taxonomy reference (IDD20k, 40 classes):
  Class ID -> class name (Level 3 hierarchy, as in IDD paper)
  Key ODD-relevant classes:
    0: road
    1: drivable fallback
    2: sidewalk/footpath
    3: non-drivable fallback
    5: person/pedestrian
    6: rider (bicycle, scooter)
    7: motorcycle/two-wheeler
    8: bicycle
    9: autorickshaw (IDD-specific)
    10: car
    11: truck
    12: bus
    13: vehicle fallback
    14: curb
    16: wall
    17: fence
    22: traffic sign
    26: animal (cattle — IDD-specific)

Output:
    data/processed/idd_lite_sample_check.png
    idd_lite_smoke_results.json

Source: IDD Lite dataset, extracted from C:/Users/toshr/Downloads/idd-lite.tar.gz
"""

import os
import json
import glob
import sys
import numpy as np
from collections import defaultdict
from PIL import Image, ImageDraw, ImageFont

# ── IDD label ID -> (name, color) for ODD-relevant classes ───────────────────
# Colors are BGR-like RGB for display
IDD_CLASSES = {
    0:  ("road",             (128, 64,  128)),
    1:  ("drivable_fallback",(81,  0,   81)),
    2:  ("sidewalk",         (244, 35,  232)),
    3:  ("nondrivable",      (152, 251, 152)),
    5:  ("person",           (220, 20,  60)),
    6:  ("rider",            (255, 0,   0)),
    7:  ("motorcycle",       (0,   0,   192)),
    8:  ("bicycle",          (119, 11,  32)),
    9:  ("autorickshaw",     (255, 204, 54)),  # IDD-specific
    10: ("car",              (0,   0,   142)),
    11: ("truck",            (0,   0,   70)),
    12: ("bus",              (0,   60,  100)),
    13: ("vehicle_fallback", (0,   80,  100)),
    14: ("curb",             (196, 196, 196)),
    16: ("wall",             (102, 102, 156)),
    17: ("fence",            (190, 153, 153)),
    22: ("traffic_sign",     (220, 220, 0)),
    26: ("animal_cattle",    (107, 142, 35)),  # IDD-specific
}

# ODD-relevant classes for this project (bold in report)
ODD_CLASSES = {9: "autorickshaw", 26: "animal_cattle", 7: "motorcycle",
               6: "rider", 5: "person", 10: "car", 11: "truck", 12: "bus"}

base = os.path.join("data", "raw", "idd_lite", "idd20k_lite")
gt_root = os.path.join(base, "gtFine")
img_root = os.path.join(base, "leftImg8bit")

# ── 1. Find all label maps ────────────────────────────────────────────────────
label_files = glob.glob(os.path.join(gt_root, "**", "*_label.png"), recursive=True)
# Exclude inst_label
label_files = [f for f in label_files if "_inst_label" not in f]
print(f"Found {len(label_files)} label maps across train/val splits")

# ── 2. Tally pixel counts per class ──────────────────────────────────────────
class_pixel_counts = defaultdict(int)
total_pixels = 0

for lf in label_files:
    arr = np.array(Image.open(lf))
    total_pixels += arr.size
    ids, counts = np.unique(arr, return_counts=True)
    for cid, cnt in zip(ids.tolist(), counts.tolist()):
        class_pixel_counts[cid] += cnt

print(f"\nTotal pixels analyzed: {total_pixels:,}")
print(f"Unique class IDs found: {sorted(class_pixel_counts.keys())}")

print("\nODD-Relevant Class Pixel Distribution:")
print(f"{'Class ID':>10} {'Name':<22} {'Pixels':>12} {'% of total':>12}")
print("-" * 58)
odd_results = {}
for cid in sorted(ODD_CLASSES.keys()):
    count = class_pixel_counts.get(cid, 0)
    pct = 100.0 * count / total_pixels if total_pixels > 0 else 0.0
    name = ODD_CLASSES[cid]
    print(f"{cid:>10} {name:<22} {count:>12,} {pct:>11.4f}%")
    odd_results[name] = {"class_id": cid, "pixel_count": count,
                         "pct_of_total": round(pct, 6)}

print("\nAll Classes:")
print(f"{'Class ID':>10} {'Name':<30} {'Pixels':>12}")
print("-" * 56)
all_results = {}
for cid in sorted(class_pixel_counts.keys()):
    count = class_pixel_counts[cid]
    name = IDD_CLASSES.get(cid, (f"class_{cid}", None))[0]
    print(f"{cid:>10} {name:<30} {count:>12,}")
    all_results[str(cid)] = {"name": name, "pixel_count": count}

# ── 3. Produce sample overlay image ──────────────────────────────────────────
# Find a label map from the train set that has at least one ODD-relevant class
sample_lf = None
for lf in label_files:
    if "train" not in lf:
        continue
    arr = np.array(Image.open(lf))
    has_odd = any(cid in arr for cid in ODD_CLASSES.keys())
    if has_odd:
        sample_lf = lf
        break

if sample_lf is None and label_files:
    sample_lf = label_files[0]

if sample_lf:
    label_arr = np.array(Image.open(sample_lf))
    H, W = label_arr.shape

    # Try to find the matching source image
    # Label path: gtFine/train/<seq>/<id>_label.png
    # Image path: leftImg8bit/train/<seq>/<id>_image.jpg (or similar)
    lf_parts = sample_lf.replace("\\", "/").split("/")
    # Extract sequence and base name
    seq_dir = lf_parts[-2]
    base_name = os.path.basename(sample_lf).replace("_label.png", "")

    img_candidates = glob.glob(os.path.join(img_root, "**", f"{base_name}*"), recursive=True)
    if not img_candidates:
        # Try to find any image in same sequence
        img_candidates = glob.glob(os.path.join(img_root, "**", seq_dir, "*.jpg"), recursive=True)

    if img_candidates:
        src_img = Image.open(img_candidates[0]).resize((W, H))
    else:
        # Create gray placeholder
        src_img = Image.fromarray(np.full((H, W, 3), 60, dtype=np.uint8))

    # Build color-coded overlay from label map
    overlay = np.zeros((H, W, 3), dtype=np.uint8)
    for cid, (name, color) in IDD_CLASSES.items():
        mask = label_arr == cid
        overlay[mask] = color

    # Blend source image with overlay
    src_arr = np.array(src_img.convert("RGB"))
    blended = (0.45 * src_arr + 0.55 * overlay).astype(np.uint8)
    result_img = Image.fromarray(blended)

    # Add legend for ODD classes present in this image
    draw = ImageDraw.Draw(result_img)
    present_classes = [cid for cid in ODD_CLASSES.keys() if cid in label_arr]

    # Header
    draw.rectangle([0, 0, W, 36], fill=(10, 10, 10, 200))
    draw.text((8, 8), f"IDD Lite Sample · {base_name} · ODD Classes present: {len(present_classes)}",
              fill=(0, 220, 200))

    # Legend
    y_off = H - 30 * len(IDD_CLASSES) - 10
    if y_off < 50:
        y_off = 50
    legend_items = [(cid, name, color) for cid, (name, color) in IDD_CLASSES.items()
                    if cid in label_arr]
    for i, (cid, name, color) in enumerate(legend_items[:20]):
        yy = y_off + i * 28
        if yy + 24 > H:
            break
        marker = [8, yy + 4, 30, yy + 22]
        is_odd = cid in ODD_CLASSES
        draw.rectangle(marker, fill=color, outline=(255, 255, 255) if is_odd else None, width=2 if is_odd else 0)
        label_text = f"[ODD] {name}" if is_odd else name
        draw.text((36, yy + 6), f"ID {cid}: {label_text}", fill=(255, 255, 255))

    out_path = os.path.join("data", "processed", "idd_lite_sample_check.png")
    result_img.save(out_path)
    print(f"\nSample overlay saved: {out_path}")
    print(f"ODD classes present in sample: {[ODD_CLASSES[c] for c in present_classes]}")
else:
    out_path = None
    print("\nWARNING: No label file found — no overlay produced")

# ── 4. Save results JSON ──────────────────────────────────────────────────────
results = {
    "source": "IDD Lite (idd20k_lite), extracted from idd-lite.tar.gz",
    "label_maps_parsed": len(label_files),
    "total_pixels_analyzed": total_pixels,
    "odd_relevant_classes": odd_results,
    "all_classes_pixel_counts": all_results,
    "sample_overlay_image": out_path,
    "note": "Pixel counts from semantic segmentation label maps. ODD classes relevant to SIH PS-26037: autorickshaw, animal_cattle, motorcycle, rider, person."
}
out_json = os.path.join("data", "processed", "idd_lite_smoke_results.json")
with open(out_json, "w") as f:
    json.dump(results, f, indent=2)
print(f"Results JSON saved: {out_json}")
print("\n=== SMOKE TEST COMPLETE ===")
