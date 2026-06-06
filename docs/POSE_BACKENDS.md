# Pose detection backends

RobotDance's video → motion extraction is a **pluggable stage**. Pose backends are registered
with their capabilities, so you can list them, compare them on your own clip, and pick one for
extraction. MediaPipe Pose is the default because it returns **3D world landmarks** needed for
retargeting; the 2D detectors are faster but need a lifting stage to drive the robot.

```bash
robotdance list-backends                       # capabilities table (dim / format / tier / retarget)
robotdance pose-compare <clip> -o out.gif      # run every available detector side-by-side
robotdance extract <clip> --backend mediapipe  # full 3D extraction (default)
robotdance extract <clip> --backend yolo11-pose+lift   # 2D detector + analytic planar lift
```

2D-only detectors are rejected for full `extract` (which needs 3D) unless you use their `*+lift`
variant. See [`robotdance_perception/backends.py`](../robotdance_perception/backends.py) for the registry.

Backends come in two **modes**: `video` (run a detector on frames — MediaPipe, YOLO11-pose,
RTMPose, and the `*+lift` variants) and `import` (a **world-grounded** external tool — `gvhmr`,
`wham` — that you run yourself, then ingest its SMPL output with `import-hmr`). The import-mode
backends recover depth and a global trajectory, which is the most reliable way to relax the
monocular depth limit; `extract --backend gvhmr` prints the redirect to that workflow. See
[`docs/RELATED_WORK.md`](RELATED_WORK.md) for how these relate to the wider landscape.

## Detector comparison

Three OSS 2D detectors on the same clip, all normalized to COCO-17 for a fair overlay:

<img src="../assets/readme/pose/pose_compare_squat.gif" width="640" alt="MediaPipe vs YOLO11-pose vs RTMPose on the same squat clip">

| backend | det rate | mean conf | ms/frame | 3D? |
| --- | --- | --- | --- | --- |
| MediaPipe (BlazePose) | 1.00 | 0.92 | 59 | ✅ world landmarks |
| YOLO11-pose (Ultralytics) | 1.00 | 0.78 | 38 | ❌ 2D only |
| RTMPose (rtmlib) | 1.00 | 0.72 | 201 | ❌ 2D only |

On this clean single-person clip all three track well (YOLO11 is fastest). MediaPipe stays the
default downstream because it also yields **3D world landmarks**; the 2D detectors would need a
2D→3D lifting stage to drive the robot. Generated with
[`scripts/compare_pose_backends.py`](../scripts/compare_pose_backends.py).

## The `*+lift` coarse baseline

For the 2D detectors there is a `*+lift` backend (`yolo11-pose+lift`, `rtmpose+lift`) that embeds
the COCO-17 pose into a **frontal plane** with an analytic anthropometric scale (torso length, which
is robust to yaw — hip width collapses when the subject turns). It is a deliberately **coarse
baseline**: it recovers *no depth*, so sagittal moves (squats) collapse while coronal moves (kata)
survive. MediaPipe's native 3D stays the default; the lift exists to make the trade-off explicit.

Quantifying it on a kata clip — native MediaPipe 3D (blue) vs YOLO11→planar lift (red), same video:

<img src="../assets/readme/pose/lift_vs_native_karate.gif" width="460" alt="native MediaPipe 3D vs planar lift on a karate kata">

| metric (147 frames, pelvis-centred) | value |
| --- | --- |
| native depth-x std | 0.175 m |
| lift depth-x std | **0.000 m** (planar by construction) |
| MPJPE native↔lift, full | 0.274 m |
| MPJPE native↔lift, frontal (y-z only) | 0.222 m |

The lift drops all forward/back motion (depth std → 0), which accounts for ~0.16 m of the 0.27 m
gap; the rest is in-plane disagreement (different detector, no perspective/yaw model). Recognisable
but coarse — exactly the honest trade-off. Generated with
[`scripts/compare_lift_vs_native.py`](../scripts/compare_lift_vs_native.py).

## End-to-end on a real robot

Both paths retargeted onto the Unitree G1 mesh (left native, right lift-only):

<img src="../assets/readme/pose/lift_vs_native_robot.gif" width="460" alt="Unitree G1 driven by native MediaPipe 3D vs by a 2D detector + planar lift">

A 2D detector alone (no native 3D) still produces a recognisable kata on the robot — retarget IK
error 0.097 m vs 0.071 m for native, ~38 % worse. Good enough to prototype with a fast 2D model,
with the depth cost made visible. Generated with
[`scripts/render_lift_vs_native_robot.py`](../scripts/render_lift_vs_native_robot.py).
