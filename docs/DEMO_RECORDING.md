# How the demo GIF was produced

`assets/demo.gif` shows the running closed loop: the vision node's annotated
frame on the left, the drive frequency commanded by the Simulink controller on
the right. No physical hardware is involved — see the status note in the main
README.

The right-hand trace is **not a screen capture of the Simulink scope**. It is
drawn by the vision node itself from the 8-byte doubles it receives back over
UDP port 12346, multiplied by the same ×50 gain the actuation layer applies. It
therefore shows exactly the values the controller sent, on the same fixed
0–150 Hz / 30 s axes as the Simulink `Frequency Scope`, and it can be
regenerated from the command line without any recording software.

---

## Reproduce it

### 1 · Start the controller

```matlab
cd control
run_controller        % loads the FIS, opens the model, enables real-time pacing
```

Press **Run** in the Simulink window (stop time is already `inf`).

`run_controller` enables *simulation pacing* at 1× wall-clock. Without it
Simulink runs this model 10–20× faster than real time, which does not matter
for control but makes any time-domain recording meaningless.

### 2 · Record 100 s of the loop driven by the cross-fade clip

```bash
python vision/make_demo_clip.py --seconds 12          # once: builds assets/dispersing.mp4
python vision/probe_detection.py --source assets/dispersing.mp4 \
    --record-demo assets/raw/demo_raw.mp4 --record-seconds 100
```

`--record-demo` writes a 1280 × 480 MP4 (`[annotated frame | frequency trace]`)
plus `demo_raw.csv` with one row per processed frame:
`t_s, freq_hz, pins, uniformity, ratio, clog`. Frames are duplicated to 30 fps
so the file plays in real time even though inference runs at a few fps.

`assets/raw/` is git-ignored; only the final GIF is tracked.

The clip is 12 s long but the vision node plays it at inference rate, so one
pile → dispersed cycle takes roughly 30–40 s of wall-clock time on a laptop
CPU. 100 s guarantees at least one complete transition.

### 3 · Pick the window and encode the GIF

Read `demo_raw.csv` to find where `freq_hz` leaves ≈91.8 Hz; start the cut
about 6 s before that and keep 16 s. For the shipped GIF the drop began at
t ≈ 31 s, so:

```bash
ffmpeg -ss 25 -t 16 -i assets/raw/demo_raw.mp4 \
       -vf "fps=12,scale=900:-1:flags=lanczos" -an assets/raw/trim.mp4

ffmpeg -i assets/raw/trim.mp4 \
       -vf "palettegen=max_colors=160:stats_mode=diff" assets/raw/palette.png

ffmpeg -i assets/raw/trim.mp4 -i assets/raw/palette.png \
       -lavfi "paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle" \
       -loop 0 assets/demo.gif
```

Result: 900 × 338 px, 16 s, 12 fps, ≈1.7 MB.

### What the GIF shows

| Time | Left pane | Right pane |
| --- | --- | --- |
| 0–6 s | Pile, `Pins: 0`, `U = 0.00`, `R = 0.00` | Flat at **91.8 Hz** |
| 6–11 s | Pins resolve one by one as the cross-fade progresses | Trace **drops** through ≈60 Hz |
| 11–16 s | ≈30 separated pins, `U ≈ 0.8`, `R ≈ 1.0` | Settles at **58.2 Hz** |

The settled value in this run is 58.2 Hz rather than the 58.4 Hz in the
results table because the live uniformity was 0.78–0.81 instead of the 0.85
measured on the still frame; the controller output responds to the feature
values it is given, which is the point.

---

## Verification checklist

- [ ] Opens and loops in a browser
- [ ] The drop from ≈92 Hz to ≈58 Hz is legible at 900 px
- [ ] The overlay's `Brain Feedback Freq` agrees with the trace in every frame
      (if it lags, a stale `python3.13.exe` was holding port 12346 — kill it and re-record)
- [ ] No file paths, user names, or window chrome anywhere in frame
- [ ] Under 5 MB
- [ ] Renders in the README on GitHub after pushing

---

## Alternative: screen-capture the Simulink scope

If you would rather show the actual Simulink `Frequency Scope` window,
arrange the OpenCV window (640 × 480, left) and the scope (≈640 × 480, right)
edge to edge at 100 % display scaling, hide every other window, and record the
region at 15 fps with [ScreenToGif](https://www.screentogif.com/) or OBS.
Start recording only once the scope has sat at ≈91.8 Hz for a few seconds and
stop ~3 s after it settles at ≈58 Hz; then resize to 900 px wide, remove
duplicate frames, and export. Everything else in this document still applies.
