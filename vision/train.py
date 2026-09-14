"""
Fine-tune YOLOv11n-seg on the probe dataset.

Reproduces the weights shipped as models/best.pt:
    yolo11n-seg.pt, 100 epochs, 640x640, single class ('probe').

    python vision/train.py

Results are written to runs/segment/probe_seg/ (git-ignored).
The pre-trained yolo11n-seg.pt checkpoint is downloaded automatically by
Ultralytics on first run.
"""

from pathlib import Path

from ultralytics import YOLO

REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_YAML = REPO_ROOT / "data" / "probe-v1-yolov11" / "data.yaml"


def main() -> None:
    model = YOLO("yolo11n-seg.pt")
    model.train(
        data=str(DATA_YAML),
        epochs=100,
        imgsz=640,
        project=str(REPO_ROOT / "runs" / "segment"),
        name="probe_seg",
        exist_ok=True,
    )
    print("\nBest weights: runs/segment/probe_seg/weights/best.pt")
    print("Copy them over models/best.pt to use them with vision/probe_detection.py")


if __name__ == "__main__":
    main()
