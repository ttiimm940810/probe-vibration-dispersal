"""
Evaluate models/best.pt on the probe dataset and print the metrics reported
in the README.

    python vision/evaluate.py
    python vision/evaluate.py --split test
"""

import argparse
from pathlib import Path

from ultralytics import YOLO

REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_YAML = REPO_ROOT / "data" / "probe-v1-yolov11" / "data.yaml"
WEIGHTS = REPO_ROOT / "models" / "best.pt"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", default="val", choices=["val", "test"])
    parser.add_argument("--weights", default=str(WEIGHTS))
    args = parser.parse_args()

    model = YOLO(args.weights)
    metrics = model.val(data=str(DATA_YAML), imgsz=640, split=args.split)

    print(f"\n--- {args.split} split ---")
    print(f"Box  mAP@50     : {metrics.box.map50 * 100:.1f} %")
    print(f"Box  mAP@50-95  : {metrics.box.map * 100:.1f} %")
    print(f"Box  Precision  : {metrics.box.mp * 100:.1f} %")
    print(f"Box  Recall     : {metrics.box.mr * 100:.1f} %")
    print(f"Mask mAP@50     : {metrics.seg.map50 * 100:.1f} %")
    print(f"Mask mAP@50-95  : {metrics.seg.map * 100:.1f} %")


if __name__ == "__main__":
    main()
