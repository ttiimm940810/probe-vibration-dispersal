"""
Closed-loop vision node for the micro-probe vibration dispersal system.

Pipeline
--------
    frame  ->  YOLOv11n-seg instance segmentation
           ->  two normalised features (uniformity U, single ratio R)
           ->  UDP TX  :12345  struct 'dd'  (16 bytes)  ->  Simulink
    Simulink  ->  UDP RX  :12346  double    ( 8 bytes)  ->  drive frequency

The receive socket is non-blocking with a 1 ms poll so the vision loop never
stalls waiting on the controller.

Usage
-----
    python vision/probe_detection.py --source data/samples/probe_dispersed.jpg
    python vision/probe_detection.py --source data/samples/probe_clogged.jpg
    python vision/probe_detection.py --source 0            # live camera
    python vision/probe_detection.py --source ... --record assets/vision_capture.mp4

    # README demo: side-by-side (annotated frame | live frequency trace), 100 s
    python vision/probe_detection.py --source assets/dispersing.mp4 \
        --record-demo assets/raw/demo_raw.mp4 --record-seconds 100

Author: Ting Lung (龍霆), NCHU BIME.
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

# --------------------------------------------------------------------------
# Configuration — these constants define the published behaviour of the system.
# Changing them changes the results reported in the README.
# --------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent

UDP_IP = "127.0.0.1"
TX_PORT = 12345          # Python -> Simulink   (Simulink UDP Receive local port)
RX_PORT = 12346          # Simulink -> Python   (Simulink UDP Send remote port)

FRAME_W, FRAME_H = 640, 480
GRID_ROWS, GRID_COLS = 4, 4      # 4x4 virtual grid for the uniformity statistic
SIGMA_MAX = 6.0                  # normalising denominator for the grid std-dev
DECAY_RATE = 0.005               # decay of the running peak count per frame
PEAK_FLOOR = 5.0                 # lower bound on the running peak
CONF_THRESHOLD = 0.15            # low on purpose: a missed pin hurts more than a false one
PROBE_CLASS_ID = 0               # single class dataset: 0 == 'probe'

FREQ_GAIN = 50.0                 # raw controller output -> Hz
FREQ_MIN_HZ, FREQ_MAX_HZ = 0.0, 150.0
RX_DRAIN_LIMIT = 20000           # max packets drained per frame (bounded loop)

WINDOW_NAME = "YOLO AI Closed-Loop Virtual Plant"

# Demo recording: right-hand panel that plots the frequency received back from
# Simulink, so the README GIF can be produced without screen-capture software.
PANEL_W, PANEL_H = 640, 480
TRACE_SECONDS = 30.0             # scrolling window, same as the Simulink scope
REC_FPS = 30                     # output video rate; frames are duplicated to real time
FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_freq_panel(history, now: float, current_hz: float, have_feedback: bool):
    """Render the frequency trace panel from (timestamp, Hz) samples.

    Axes match the Simulink Frequency Scope: 0-150 Hz fixed, 30 s scrolling
    window. Values are exactly what arrived over UDP :12346 multiplied by the
    same x50 gain the actuation layer applies — nothing is synthesised.
    """
    panel = np.full((PANEL_H, PANEL_W, 3), (22, 22, 22), np.uint8)
    x0, x1 = 64, PANEL_W - 22
    y0, y1 = 58, PANEL_H - 44

    for hz in range(0, 151, 30):                       # horizontal grid
        y = int(round(y1 - hz / 150.0 * (y1 - y0)))
        cv2.line(panel, (x0, y), (x1, y), (58, 58, 58), 1)
        cv2.putText(panel, f"{hz:3d}", (x0 - 42, y + 5), FONT, 0.45, (170, 170, 170), 1)
    for s in range(0, 31, 5):                          # vertical grid, -30 .. 0 s
        x = int(round(x0 + s / TRACE_SECONDS * (x1 - x0)))
        cv2.line(panel, (x, y0), (x, y1), (58, 58, 58), 1)
        cv2.putText(panel, f"{s - 30:d}", (x - 10, y1 + 18), FONT, 0.42, (170, 170, 170), 1)
    cv2.rectangle(panel, (x0, y0), (x1, y1), (110, 110, 110), 1)

    pts = []
    for t, hz in history:
        age = now - t
        if age > TRACE_SECONDS:
            continue
        pts.append((int(round(x1 - age / TRACE_SECONDS * (x1 - x0))),
                    int(round(y1 - hz / 150.0 * (y1 - y0)))))
    if len(pts) >= 2:
        cv2.polylines(panel, [np.array(pts, np.int32)], False, (0, 215, 255), 2)

    cv2.putText(panel, "Commanded drive frequency", (x0, 24), FONT, 0.62, (235, 235, 235), 1)
    cv2.putText(panel, "received from Simulink controller, UDP :12346, x50 gain",
                (x0, 44), FONT, 0.42, (150, 150, 150), 1)
    value = f"{current_hz:5.1f} Hz" if have_feedback else "  --  Hz"
    cv2.putText(panel, value, (x1 - 128, 30), FONT, 0.75, (0, 215, 255), 2)
    cv2.putText(panel, "Hz", (x0 - 50, y0 - 8), FONT, 0.45, (170, 170, 170), 1)
    cv2.putText(panel, "time (s)", (x1 - 62, PANEL_H - 8), FONT, 0.45, (170, 170, 170), 1)
    return panel


def imread_unicode(path: Path):
    """cv2.imread replacement that works when the path contains non-ASCII
    characters. On Windows cv2.imread goes through the ANSI file API and can
    return None for such paths depending on the active code page — easy to hit
    when the repository lives under a localised folder name. Reading the bytes
    ourselves and decoding from memory avoids that entirely.
    """
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


# --------------------------------------------------------------------------
# Feature extraction
# --------------------------------------------------------------------------
def compute_features(grid_counts: np.ndarray, single_count: int, peak: float
                     ) -> tuple[float, float, float, bool]:
    """Turn one frame's detections into the two controller inputs.

    Returns
    -------
    uniformity : float in [0, 1]   1.0 == pins evenly spread over the 4x4 grid
    ratio      : float in [0, 1]   1.0 == every pin currently resolvable as a single
    peak       : float             updated running peak count
    clogged    : bool              entanglement guard tripped
    """
    # Entanglement guard: pins were being tracked and the count collapsed to zero.
    # That is not an empty tray, it is a pile the segmenter can no longer resolve.
    if single_count == 0 and peak > 10.0:
        return 0.0, 0.0, peak, True

    # Running peak with slow decay, so the metric adapts to the batch actually
    # in the tray instead of assuming a fixed pin count.
    if float(single_count) > peak:
        peak = float(single_count)
    else:
        peak = max(peak * (1.0 - DECAY_RATE), float(single_count), PEAK_FLOOR)

    ratio = float(np.clip(single_count / peak, 0.0, 1.0))

    if single_count > 0:
        sigma = float(np.std(grid_counts))
        uniformity = float(np.clip(1.0 - sigma / SIGMA_MAX, 0.0, 1.0))
    else:
        uniformity = 0.0

    return uniformity, ratio, peak, False


# --------------------------------------------------------------------------
# Frame source
# --------------------------------------------------------------------------
class FrameSource:
    """Yields frames from a still image (looped), a video file, or a camera index."""

    def __init__(self, source: str):
        self.capture = None
        self.still = None

        if source.isdigit():
            self.capture = cv2.VideoCapture(int(source))
            self.kind = "camera"
        else:
            path = Path(source)
            if not path.is_absolute():
                path = REPO_ROOT / path
            if not path.exists():
                raise FileNotFoundError(f"Source not found: {path}")
            if path.suffix.lower() in {".mp4", ".avi", ".mov", ".mkv"}:
                self.capture = cv2.VideoCapture(str(path))
                self.kind = "video"
            else:
                self.still = imread_unicode(path)
                if self.still is None:
                    raise ValueError(f"Could not decode image: {path}")
                self.still = cv2.resize(self.still, (FRAME_W, FRAME_H))
                self.kind = "image"

        if self.capture is not None and not self.capture.isOpened():
            raise RuntimeError(f"Could not open source: {source}")

    def read(self):
        if self.still is not None:
            return self.still.copy()
        ok, frame = self.capture.read()
        if not ok:
            if self.kind == "video":            # loop the clip
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.capture.read()
            if not ok:
                return None
        return cv2.resize(frame, (FRAME_W, FRAME_H))

    def release(self):
        if self.capture is not None:
            self.capture.release()


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default="data/samples/probe_dispersed.jpg",
                        help="image / video path, or a camera index such as 0")
    parser.add_argument("--weights", default="models/best.pt",
                        help="path to the trained YOLOv11n-seg weights")
    parser.add_argument("--record", default=None,
                        help="write the annotated window to this MP4")
    parser.add_argument("--record-demo", default=None,
                        help="write a 1280x480 MP4: annotated frame | live frequency "
                             "trace, plus a .csv of (t, Hz, pins, U, R) beside it")
    parser.add_argument("--record-seconds", type=float, default=0.0,
                        help="stop automatically after this many seconds (0 = until 'q')")
    parser.add_argument("--no-udp", action="store_true",
                        help="run the vision layer alone, without the Simulink link")
    args = parser.parse_args()

    weights = Path(args.weights)
    if not weights.is_absolute():
        weights = REPO_ROOT / weights
    if not weights.exists():
        print(f"[error] weights not found: {weights}", file=sys.stderr)
        return 1

    model = YOLO(str(weights))
    source = FrameSource(args.source)

    sock_tx = sock_rx = None
    if not args.no_udp:
        sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_tx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock_rx.bind((UDP_IP, RX_PORT))
        sock_rx.setblocking(False)

    writer = None
    if args.record:
        out_path = Path(args.record)
        if not out_path.is_absolute():
            out_path = REPO_ROOT / out_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(out_path),
                                 cv2.VideoWriter_fourcc(*"mp4v"), 30, (FRAME_W, FRAME_H))

    demo_writer = None
    csv_file = csv_writer = None
    if args.record_demo:
        demo_path = Path(args.record_demo)
        if not demo_path.is_absolute():
            demo_path = REPO_ROOT / demo_path
        demo_path.parent.mkdir(parents=True, exist_ok=True)
        demo_writer = cv2.VideoWriter(str(demo_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                      REC_FPS, (FRAME_W + PANEL_W, FRAME_H))
        csv_file = open(demo_path.with_suffix(".csv"), "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["t_s", "freq_hz", "pins", "uniformity", "ratio", "clog"])

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, FRAME_W + (PANEL_W if demo_writer else 0), FRAME_H)

    peak = 10.0
    last_valid_freq = 0.0
    have_feedback = False
    history: deque = deque()          # (monotonic time, Hz)
    rec_start = time.monotonic()
    last_write_t = None

    udp_state = "off" if args.no_udp else f"tx :{TX_PORT} / rx :{RX_PORT}"
    print(f"[ok] source={source.kind} ({args.source})  udp={udp_state}   press 'q' to quit")

    try:
        while True:
            frame = source.read()
            if frame is None:
                break

            results = model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
            canvas = frame.copy()

            single_count = 0
            grid_counts = np.zeros((GRID_ROWS, GRID_COLS))

            if results.masks is not None:
                classes = results.boxes.cls.cpu().numpy()
                for idx, mask in enumerate(results.masks.xy):
                    if int(classes[idx]) != PROBE_CLASS_ID:
                        continue
                    single_count += 1

                    cv2.polylines(canvas, [np.array(mask, dtype=np.int32)],
                                  isClosed=True, color=(0, 255, 0), thickness=2)

                    # mask centroid -> grid cell
                    cx = float(np.mean(mask[:, 0]))
                    cy = float(np.mean(mask[:, 1]))
                    col = min(int(cx / (FRAME_W / GRID_COLS)), GRID_COLS - 1)
                    row = min(int(cy / (FRAME_H / GRID_ROWS)), GRID_ROWS - 1)
                    grid_counts[row, col] += 1

            uniformity, ratio, peak, clogged = compute_features(grid_counts, single_count, peak)

            # ---- TX: two doubles, 16 bytes, to the Simulink UDP Receive block ----
            if sock_tx is not None:
                sock_tx.sendto(struct.pack("dd", uniformity, ratio), (UDP_IP, TX_PORT))

            # ---- RX: one double, 8 bytes, non-blocking ----
            # Simulink transmits on every solver step (hundreds to thousands of
            # packets per second); this loop runs at camera rate. Reading a
            # single packet per frame would return the OLDEST packet in the
            # socket buffer and the overlay would lag further behind the scope
            # every second. Drain the buffer and keep only the newest packet.
            feedback_hz = last_valid_freq
            if sock_rx is not None:
                newest = None
                for _ in range(RX_DRAIN_LIMIT):
                    try:
                        data, _ = sock_rx.recvfrom(1024)
                    except (BlockingIOError, socket.timeout, OSError):
                        break
                    if len(data) == 8:
                        newest = data
                if newest is not None:
                    raw_val = struct.unpack("d", newest)[0]
                    if not math.isnan(raw_val) and not math.isinf(raw_val):
                        hz = float(raw_val) * FREQ_GAIN
                        if FREQ_MIN_HZ <= hz <= FREQ_MAX_HZ:
                            feedback_hz = hz
                            last_valid_freq = hz
                            have_feedback = True

            now = time.monotonic()
            if have_feedback:
                history.append((now, feedback_hz))
                while history and now - history[0][0] > TRACE_SECONDS:
                    history.popleft()

            # ---- overlay ----
            for i in range(1, GRID_COLS):
                x = int(FRAME_W / GRID_COLS * i)
                cv2.line(canvas, (x, 0), (x, FRAME_H), (100, 100, 100), 1)
            for i in range(1, GRID_ROWS):
                y = int(FRAME_H / GRID_ROWS * i)
                cv2.line(canvas, (0, y), (FRAME_W, y), (100, 100, 100), 1)

            cv2.putText(canvas, f"Uniformity(In1): {uniformity:.2f} | Ratio(In2): {ratio:.2f}",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(canvas, f"Brain Feedback Freq: {feedback_hz:.1f} Hz",
                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(canvas, f"Pins: {single_count}   State: {'CLOG ALERT' if clogged else 'NORMAL'}",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 255) if clogged else (0, 255, 255), 2)

            print(f"state={'ALERT ' if clogged else 'NORMAL'} | pins={single_count:2d} | "
                  f"U={uniformity:.2f} | R={ratio:.2f} | freq={feedback_hz:6.1f} Hz",
                  end="\r", flush=True)

            if writer is not None:
                writer.write(canvas)

            shown = canvas
            if demo_writer is not None:
                panel = draw_freq_panel(history, now, feedback_hz, have_feedback)
                shown = np.hstack([canvas, panel])
                # The loop runs at inference rate (well under 30 fps); duplicate
                # frames so the file plays back in real time.
                n = 1 if last_write_t is None else max(1, int(round((now - last_write_t) * REC_FPS)))
                for _ in range(min(n, REC_FPS * 2)):
                    demo_writer.write(shown)
                last_write_t = now
                csv_writer.writerow([f"{now - rec_start:.3f}", f"{feedback_hz:.2f}", single_count,
                                     f"{uniformity:.3f}", f"{ratio:.3f}", int(clogged)])

            cv2.imshow(WINDOW_NAME, shown)

            if args.record_seconds and (now - rec_start) >= args.record_seconds:
                print(f"\n[done] recorded {args.record_seconds:.0f} s")
                break
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[stop] interrupted by user")
    finally:
        print()
        if writer is not None:
            writer.release()
        if demo_writer is not None:
            demo_writer.release()
        if csv_file is not None:
            csv_file.close()
        source.release()
        cv2.destroyAllWindows()
        if sock_tx is not None:
            sock_tx.close()
        if sock_rx is not None:
            sock_rx.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
