"""
Build a short clip that cross-fades from the piled/entangled frame to the
dispersed frame, so the closed loop can be demonstrated end to end without a
physical vibratory plate.

    python vision/make_demo_clip.py                    # 10 s, 30 fps
    python vision/make_demo_clip.py --seconds 6

Then feed it to the vision node:

    python vision/probe_detection.py --source assets/dispersing.mp4

Note: this is a visual stand-in for the dispersal process, not a physical
experiment. It exists so the frequency command can be seen responding to a
changing probe state; the pins in the clip do not actually move in response to
the command.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
CLOGGED = REPO_ROOT / "data" / "samples" / "probe_clogged.jpg"
DISPERSED = REPO_ROOT / "data" / "samples" / "probe_dispersed.jpg"
FRAME_W, FRAME_H = 640, 480


def imread_unicode(path: Path):
    """cv2.imread replacement that works when the path contains non-ASCII
    characters. On Windows cv2.imread goes through the ANSI file API and
    returns None for such paths depending on the active code page, which is
    easy to hit when the repository lives under a localised folder name.
    Reading the bytes ourselves and decoding from memory avoids that entirely.
    """
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="assets/dispersing.mp4")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    a = imread_unicode(CLOGGED)
    b = imread_unicode(DISPERSED)
    if a is None or b is None:
        missing = [str(p) for p, img in ((CLOGGED, a), (DISPERSED, b)) if img is None]
        raise SystemExit("Could not read:\n  " + "\n  ".join(missing))

    a = cv2.resize(a, (FRAME_W, FRAME_H))
    b = cv2.resize(b, (FRAME_W, FRAME_H))

    out_path = Path(args.out)
    if not out_path.is_absolute():
        out_path = REPO_ROOT / out_path
    out_path.parent.mkdir(parents=True, exist_ok=True)

    writer = cv2.VideoWriter(str(out_path),
                             cv2.VideoWriter_fourcc(*"mp4v"), args.fps,
                             (FRAME_W, FRAME_H))

    total = int(args.seconds * args.fps)
    hold = int(total * 0.15)          # hold each end state so the controller settles

    for i in range(total):
        if i < hold:
            alpha = 0.0
        elif i > total - hold:
            alpha = 1.0
        else:
            alpha = (i - hold) / max(total - 2 * hold, 1)
        writer.write(cv2.addWeighted(a, 1.0 - alpha, b, alpha, 0))

    writer.release()
    print(f"wrote {out_path}  ({total} frames, {args.seconds:.1f} s @ {args.fps} fps)")


if __name__ == "__main__":
    main()
