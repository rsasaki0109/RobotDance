<div align="center">

# 🕺 RobotDance

**Human Video → Humanoid Motion Compiler**

*An OSS compiler that turns human videos into motion humanoids can learn, search, imitate, and execute.*

English · [**日本語**](README.ja.md)

[![CI](https://github.com/rsasaki0109/RobotDance/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/RobotDance/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
![Python](https://img.shields.io/badge/python-3.10+-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E.svg)

<img src="assets/readme/karate_hero.gif" width="560" alt="Real karate video with skeleton overlay on the left, Unitree G1 reproducing the same kata on the right">

<sub>A real karate video (left, 2D skeleton overlaid on the footage) → a real Unitree G1 reproduces the same kata on the right. That's "Shorts to humanoid" in one line. Source: Sdcsabac, CC BY-SA 4.0 (Wikimedia); raw video is not redistributed (renders only).</sub>

### 🎬 Many motions × three robots

<table>
<tr>
<td align="center"><sub><b>G1</b><br>1.29m</sub></td>
<td align="center"><img src="assets/readme/gallery/g1_groove.gif" width="120" alt="G1 groove"><br><sub>groove</sub></td>
<td align="center"><img src="assets/readme/gallery/g1_fast.gif" width="120" alt="G1 fast"><br><sub>fast</sub></td>
<td align="center"><img src="assets/readme/gallery/g1_wave.gif" width="120" alt="G1 wave"><br><sub>wave</sub></td>
<td align="center"><img src="assets/readme/gallery/g1_march.gif" width="120" alt="G1 march"><br><sub>march</sub></td>
<td align="center"><img src="assets/readme/gallery/g1_squat.gif" width="120" alt="G1 squat"><br><sub>squat</sub></td>
</tr>
<tr>
<td align="center"><sub><b>H1</b><br>1.66m</sub></td>
<td align="center"><img src="assets/readme/gallery/h1_groove.gif" width="120" alt="H1 groove"><br><sub>groove</sub></td>
<td align="center"><img src="assets/readme/gallery/h1_fast.gif" width="120" alt="H1 fast"><br><sub>fast</sub></td>
<td align="center"><img src="assets/readme/gallery/h1_wave.gif" width="120" alt="H1 wave"><br><sub>wave</sub></td>
<td align="center"><img src="assets/readme/gallery/h1_march.gif" width="120" alt="H1 march"><br><sub>march</sub></td>
<td align="center"><img src="assets/readme/gallery/h1_squat.gif" width="120" alt="H1 squat"><br><sub>squat</sub></td>
</tr>
<tr>
<td align="center"><sub><b>H2</b><br>1.76m</sub></td>
<td align="center"><img src="assets/readme/gallery/h2_groove.gif" width="120" alt="H2 groove"><br><sub>groove</sub></td>
<td align="center"><img src="assets/readme/gallery/h2_fast.gif" width="120" alt="H2 fast"><br><sub>fast</sub></td>
<td align="center"><img src="assets/readme/gallery/h2_wave.gif" width="120" alt="H2 wave"><br><sub>wave</sub></td>
<td align="center"><img src="assets/readme/gallery/h2_march.gif" width="120" alt="H2 march"><br><sub>march</sub></td>
<td align="center"><img src="assets/readme/gallery/h2_squat.gif" width="120" alt="H2 squat"><br><sub>squat</sub></td>
</tr>
</table>

<sub>The same choreography retargeted onto real G1 / H1 / H2 meshes — differences in height and DOF show through directly.<br>
※ Meshes © Unitree Robotics (BSD-3-Clause, not bundled in this repo). GIFs are visualizations of pipeline output.</sub>

</div>

---

## What is this?

```
Input:  a short human video (synthetic / real video (MediaPipe) / mocap (AMASS))
Output: robot-executable motion + RD-MIR dataset + motion embedding
```

Every input converges to the canonical **RD-MIR** (the core motion IR) and flows through **retarget → physics check → embedding → ROS2 safe playback**.

| ✅ RobotDance is | ❌ is not |
| --- | --- |
| a **motion compiler**: video → humanoid motion assets | a TikTok/Instagram scraper |
| a data OS that standardizes **RD-MIR** | just a pose-estimation wrapper |
| a **sim-first** stack targeting G1/H1 first | a "drop a video, robot dances now" toy |
| a **frontend** feeding motion priors to Isaac Lab etc. | a competitor to Isaac Lab / GR00T |

> ⚠️ v0 is pre-alpha and includes approximations — **not a guarantee on real hardware** (mass/inertia come from real URDFs, but balance/torque use a quasi-static approximation). The boundary is documented in [`docs/SIM_TO_REAL.md`](docs/SIM_TO_REAL.md).

## Real video → humanoid (the headline "Shorts to humanoid")

Recover 3D from a local video with MediaPipe Pose, then go end-to-end: RD-MIR → retarget → physics check. Three stages — **① skeleton overlay → ② canonical skeleton → ③ real G1**:

<table>
<tr>
<td align="center"><img src="assets/readme/real/karate3_g1_overlay.gif" width="220" alt="2D skeleton overlay on a karate kata video"><br><sub>① source video + skeleton overlay</sub></td>
<td align="center"><img src="assets/readme/real/karate3_g1_skeleton.gif" width="170" alt="canonical skeleton"><br><sub>② RD-MIR skeleton</sub></td>
<td align="center"><img src="assets/readme/real/karate3_g1_robot.gif" width="180" alt="G1 mesh performing the karate kata stance"><br><sub>③ real G1 reproduces it</sub></td>
</tr>
</table>

<sub>All three stages come from one extract — the forward stance and arm techniques line up across the overlay, the recovered skeleton, and the robot. The robot is dynamically grounded each frame (lowest point on the floor), so it bends and steps with the human instead of floating at a fixed pelvis height. The actuator-IK retarget weights **end-effectors (hands/feet) over the near-rigid shoulders/hips**, so strikes extend instead of collapsing into a crouch, and an optional **occlusion guard** (`retarget-ik --conf-gate`) holds a limb's last confident direction when monocular detection drops out (e.g. the far arm in a side view).</sub>

**More clips → real G1 / H1:**

<table>
<tr>
<td align="center"><img src="assets/readme/real/squat_g1_robot.gif" width="150" alt="G1 squat"><br><sub>squat → G1</sub></td>
<td align="center"><img src="assets/readme/real/kathak3_g1_robot.gif" width="150" alt="G1 kathak dance"><br><sub>kathak → G1</sub></td>
<td align="center"><img src="assets/readme/real/kathak3_h1_robot.gif" width="150" alt="H1 kathak dance"><br><sub>kathak → H1</sub></td>
</tr>
</table>

<sub>※ **Source videos are not bundled in this repo.** Only the overlay is a derivative containing source pixels (allowed under CC-BY with attribution); the rest visualize the extracted motion and contain no source pixels. Sources (Wikimedia Commons): karate kata — Sdcsabac (CC BY-SA 4.0); kathak — Suyash Dwivedi (CC BY-SA 4.0); squat (clip above) — Taco Fleur (CC BY-SA 4.0); squat (detector & physics demos below) — FitnessScape (CC BY 3.0). Generated with [`scripts/render_real_video_gif.py`](scripts/render_real_video_gif.py).</sub>

### Pose detection — swap in different OSS detectors

Extraction is a pluggable stage. MediaPipe Pose is the default (it returns the **3D world landmarks** needed for retargeting), but backends are registered with their capabilities, so you can `list-backends`, `pose-compare <clip>` them side-by-side, and `extract --backend <name>`. Three OSS 2D detectors on the same clip, normalized to COCO-17:

<img src="assets/readme/pose/pose_compare_squat.gif" width="640" alt="MediaPipe vs YOLO11-pose vs RTMPose on the same squat clip">

| backend | det rate | mean conf | ms/frame | 3D? |
| --- | --- | --- | --- | --- |
| MediaPipe (BlazePose) | 1.00 | 0.92 | 59 | ✅ world landmarks |
| YOLO11-pose (Ultralytics) | 1.00 | 0.78 | 38 | ❌ 2D only |
| RTMPose (rtmlib) | 1.00 | 0.72 | 201 | ❌ 2D only |

<sub>The 2D detectors can still drive the robot via a `*+lift` coarse baseline (analytic 2D→frontal-plane lift, no depth) — a YOLO11-only kata reaches 0.097 m retarget IK error vs 0.071 m for native. Full comparison, metrics and robot demo: **[docs/POSE_BACKENDS.md](docs/POSE_BACKENDS.md)**.</sub>

### The physics check is the safety valve — it stops infeasible motion

Feed the extracted real squat into the feasibility certificate (real URDF inertia) and it **REJECTs**. The reasons are diagnostic, by design stopping "drop a video, robot dances now". `--ground-clean` (locking the contact foot to z=0) removes contact artifacts, but **the remaining balance is limited by monocular depth error**:

<table>
<tr><td>

| axis | raw extraction | --ground-clean |
| --- | --- | --- |
| airborne | ⛔ 0.484 | ✅ **0.000** |
| torque | ✅ 0.878 | ✅ **0.615** |
| balance | ⛔ 0.601 | ⛔ **0.474** |
| **verdict** | REJECT | REJECT (balance remains) |

</td><td>

<img src="assets/readme/real/squat_g1_balance_cleaned.png" width="250" alt="ZMP vs support polygon: residual spread in depth axis">

</td></tr>
</table>

<sub>The residual ZMP excursion concentrates along the forward x axis (depth — the least reliable axis in monocular). A full PASS needs better depth estimation / contact-aware retargeting — v0's honest frontier.</sub>

### Benchmark — why each motion passes or fails

`robotdance benchmark --chart` runs the motion suite × robots and plots every run by **torque ratio (×actuator limit)** vs **balance-violation ratio**, so you can see *which axis* each motion is limited by:

<img src="assets/readme/benchmark_feasibility.png" width="560" alt="Feasibility scatter: torque ratio vs balance violation, PASS/REJECT per motion and robot">

<sub>40 runs (8 motions × 5 robots). PASS (green) cluster in the feasible region (torque ≤ 1.0, low balance violation). Failures split by cause: `backflip` / `march` are **balance-limited** (top), `dance_fast` is **torque-limited** (right). Marker = embodiment (G1/H1/H2/T1/Apollo) — the same motion has different feasibility per robot. Generated with `benchmark --chart` (MuJoCo). Per-motion / per-robot tables: `LEADERBOARD.md`.</sub>

**Embodiment reach fidelity** — even when bone *directions* are preserved (cos ≈ 1.0 for every robot), the height-normalized hand/foot **reach error** differs by embodiment because limb proportions don't match the human. It is purely geometric (no physics), so `benchmark --no-sim` reports it for any motion:

| robot | bone-dir cos | reach error (height-normalized) |
| --- | --- | --- |
| Booster T1 | 1.00 | **0.116 m** |
| Unitree H2 | 1.00 | 0.117 m |
| Unitree G1 | 1.00 | 0.121 m |
| Apptronik Apollo | 1.00 | 0.139 m |
| Unitree H1 | 1.00 | **0.146 m** |

<sub>40 runs (8 motions × 5 robots), `benchmark --no-sim`. Direction fidelity alone says "every robot is perfect"; reach error exposes the limb-proportion gap the cosine hides — H1's long limbs drift most, T1's compact build least. This is the geometric ceiling of direction-preserving retarget, independent of physics feasibility above.</sub>

```bash
pip install -e ".[demo,sim,perception]"

robotdance video-to-robot my_clip.mp4 --robot unitree_g1 -o out.gif      # video → check → side-by-side
robotdance extract my_clip.mp4 -o clip.rdmir.json                        # video → RD-MIR
robotdance motion-doctor clip.rdmir.json                                 # health check (mirror/depth/grounding)
robotdance overlay my_clip.mp4 clip.rdmir.json -o overlay.gif            # skeleton overlay
robotdance validate-sim clip.rdmir.json --robot unitree_g1 --ground-clean --balance-plot b.png  # physics check
```

## Quick start (no external models or licensed videos needed)

```bash
pip install -e ".[demo,sim]"

robotdance demo-multi  -o many_humanoids.gif --robots unitree_g1 unitree_h1  # same motion, many robots
robotdance demo-safety -o safety_check.gif --robot unitree_g1               # safe(PASS) vs backflip(REJECT)
robotdance synth -o dance.rdmir.json --duration 4                           # synthetic RD-MIR
robotdance validate-sim dance.rdmir.json --robot unitree_g1                 # physics check (executable: yes/no)
```

## What it can do

Inputs (synthetic / real video / mocap) → RD-MIR → the pipeline below. See `--help` and each package README for details on every `command`.

<details><summary><b>Feature list (click to expand)</b></summary>

| area | main commands |
| --- | --- |
| extraction | `extract` (`--backend`) `import-hmr` `import-humanml3d` `import-babel` `import-motionx` `smooth` `overlay` |
| pose backends & QC | `list-backends` (mediapipe / 2D+lift / gvhmr·wham) `pose-compare` `motion-doctor` (mirror/depth/grounding) |
| dataset | `build-dataset` (RD-Manifest + license firewall / Data BOM) `dedupe-dir` |
| retarget | `retarget` `retarget-ik` (real G1 23 joint angles, end-effector-weighted, `--conf-gate` occlusion guard) `export-joints` (joint-angle CSV/JSON for real-robot/sim SDKs) `list-retargeters` (builtin / GMR) `demo-multi` (G1/H1/T1/Apollo) |
| physics check | `validate-sim` (sim_certificate, MuJoCo) `--ground-clean` `--balance-plot` `sim-backends` |
| embedding & search | `demo-motion-map` `train-encoder` `train-text-motion` `search-text` `search-motion` (`--text` zero-dep concept search, `--healthy-only` quality-aware) |
| generation | `train-tokenizer` (VQ-VAE) `train-prior` `demo-generate` `train-text2motion` `generate-text` `train-denoiser` |
| learned policy | `train-tracking` (PPO) `demo-track` `demo-track-multi` `export-policy` (RD-Policy + ONNX) |
| benchmark | `benchmark` (motion×robot leaderboard) `benchmark-extraction` |
| cards | `model-card` `cards-index` (lineage/license/failure/safety) |
| ROS2 runtime | `serve --ros2` (estop / pause / seek topics) `demo-runtime` (safety guard) `demo-joint-safety` |
| integration | `demo-pipeline` (RD-MIR→retarget→sim→policy→cards in one command) |
| specs | `validate` (RD-MIR/Manifest/… schema 検証) `specs` (spec registry + versions) |

</details>

<details><summary><b>Embedding, search, generation (with images)</b></summary>

**Motion Map** — encode RD-MIR into embeddings for similarity search, near-duplicate removal, and a 2D map:

<img src="assets/readme/motion_map.png" width="380">

```bash
robotdance demo-motion-map -o motion_map.png
robotdance train-text-motion -o tm.pt && robotdance search-text "a backflip" --checkpoint tm.pt
# zero-dependency text search — no checkpoint, matches each motion's action_label by concept
# (synonyms/morphology folded: "doing a somersault" → backflip, "upbeat dance" → energetic dance):
robotdance search-motion ./corpus --text "doing a somersault" -k 3
robotdance generate-text "a person doing a backflip" -o bf.rdmir.json --gif bf.gif
```

Generated outputs are schema-conformant RD-MIR, so they flow straight into retarget → physics check → ROS2 safe playback. v0 uses a small synthetic corpus with a limited vocabulary, so **generated motion is not guaranteed physically valid** (always verify with `validate-sim`).

</details>

## Design pillars

- **license-safe**: raw video/mocap/meshes are **never redistributed** (URL/manifest + local rebuild). A firewall blocks publishing derived motion when `license_state=unknown`. SMPL is optional.
- **sim-first**: every retargeted motion is feasibility-checked in MuJoCo physics; infeasible motion is rejected. A real-hardware bridge comes only after safety review.
- **ROS2 (Jazzy)**: only certified `.rdmotion` is streamed through the safety guard (real URDF visualized in RViz via `/joint_states`).

| license target | policy |
| --- | --- |
| Code | Apache-2.0 |
| Schema / manifest | CC0 or Apache-2.0 |
| Model weights | split into open / research-only / non-distributed |

## Supported robots

Retarget + physics check onto **Unitree G1 · H1 · H2 / Booster T1 / Apptronik Apollo** with morphology (mass/inertia/joint limits) derived from real URDFs. Provenance in [`docs/EMBODIMENTS.md`](docs/EMBODIMENTS.md).

## Repository layout

```
specs/             specs (RD-Manifest / RD-MIR / RD-Embodiment / RD-Motion / RD-Policy)
robotdance_core/        schemas, validators, CLI        robotdance_models/    tokenizer/encoder/policy
robotdance_data/        adapters, dataset, firewall     robotdance_ros2/      motion server, safety guard
robotdance_perception/  pose / HMR, smoothing           robotdance_unitree/   URDF map, SDK2/ROS2 bridge
robotdance_motion/      canonical, contacts, embeddings robotdance_benchmarks/ leaderboard
robotdance_retarget/    retargeting                     robotdance_viewer/    visualization
robotdance_sim/         MuJoCo / Isaac Lab backend
```

## Status

pre-alpha (latest version and full changelog in [CHANGELOG](CHANGELOG.md)). Working: specs v0, extraction (MediaPipe/HMR), dataset, embedding/generation, retarget (real URDF), MuJoCo physics check, RL tracking, ROS2 runtime, and benchmark. Roadmap in [`docs/ROADMAP.md`](docs/ROADMAP.md).

How RobotDance relates to GMR, GVHMR/WHAM, H2O/OmniH2O, PHC, PHUMA and the rest of the *human-video → humanoid* landscape — and what makes it different (a license-safe, feasibility-gated compiler) — is mapped in [`docs/RELATED_WORK.md`](docs/RELATED_WORK.md).

## License

Code is [Apache-2.0](LICENSE). Verify dataset/model usage terms separately per source.
