# Dataset card — `probe-v1-yolov11`

## Summary

| | |
| --- | --- |
| Task | Instance segmentation, single class `probe` |
| Images | **46** total — 32 train / 9 validation / 5 test |
| Background images | 9 of the 46 contain no pins (empty label files), included deliberately as negatives |
| Annotation format | YOLOv11 segmentation polygons (`class cx cy w h x1 y1 x2 y2 ...`, normalised) |
| Export resolution | 432 × 432, auto-orientation applied, stretch resize |
| Training resolution | 640 × 640 (`imgsz=640`) |
| Augmentation at export | None — only Ultralytics' built-in training-time augmentation is used |
| Annotation tool | Roboflow |

## Provenance

All images were photographed by the author for this project: loose semiconductor
probe pins (1.5–3.5 mil / ≈38–89 µm diameter) placed on a plain white tray under
ambient lighting, captured with a consumer camera from directly above.

The images show **only** probe pins on a featureless tray. They contain no
equipment, no fixtures, no process parameters, no markings, and no third-party
material of any kind.

Capture sessions are encoded in the file names (`20260428_1443xx`, `20260428_1916xx`),
which is why the split is not fully session-disjoint — see the limitation note below.

## Class definition

| id | name | annotated as |
| --- | --- | --- |
| 0 | `probe` | one **individually resolvable** pin |

Pins that are so entangled that their outline cannot be traced separately are
**not** annotated. This is intentional and is what makes the *single ratio*
feature meaningful: a drop in the detected count is the observable signature of
entanglement, and the controller reacts to exactly that.

## Known limitations

- **Small.** 46 images from two capture sessions on one tray under one lighting
  setup. Sufficient to train a working detector for this setup; not sufficient to
  claim generalisation.
- **Split is by frame, not by session.** Frames from the same session appear in
  more than one split, so the validation and test scores are optimistic relative
  to a session-disjoint split.
- **Boundary precision.** The pins are only a few pixels wide, so mask contours
  are loose at high IoU thresholds (Mask mAP@50-95 = 45.1 %). The uniformity
  feature uses mask *centroids*, which is comparatively robust to this.

## Licence

Released under **CC BY 4.0**, matching the original Roboflow export. Attribution:
Ting Lung (龍霆), NCHU BIME, 2026.

## Regenerating

```bash
python vision/train.py       # 100 epochs, 640x640, yolo11n-seg
python vision/evaluate.py    # metrics on the validation split
```
