# Related work — where RobotDance sits

A map of the research and OSS around *human video → humanoid motion*, and how RobotDance
positions itself. Surveyed June 2026; pointers are starting points, not endorsements.

RobotDance is a **license-safe, feasibility-gated motion compiler**: it standardizes an
intermediate representation (RD-MIR), retargets to real robot URDFs, and gates everything
through a transparent physics certificate — rather than training a controller or shipping the
largest dataset. The neighbours below each do part of this pipeline better; this page is about
borrowing from them where it helps and being honest about the gaps.

## 1. Video → 3D human recovery (the extraction front-end)

RobotDance's default extractor is MediaPipe Pose (root-relative world landmarks, **no global
trajectory**, depth is the least reliable axis — see [SIM_TO_REAL.md](SIM_TO_REAL.md)). The
state of the art recovers **world-grounded** SMPL from monocular video:

- **GVHMR** (ZJU, SIGGRAPH Asia 2024) — predicts pose in gravity-view coordinates, robust global
  motion on long clips. Code + weights public. <https://zju3dv.github.io/gvhmr/>
- **WHAM** (CVPR 2024) and **TRAM** (ECCV 2024) — combine human mesh recovery with SLAM-based
  camera tracking to place SMPL in world coordinates.

**Relevance:** these directly attack RobotDance's "depth-limited" frontier. RobotDance ingests
their output via `import-hmr` (4DHumans / GVHMR SMPL → RD-MIR), and as of v0.94 **`gvhmr` and
`wham` are registered `import`-mode backends** in the pose registry
([POSE_BACKENDS.md](POSE_BACKENDS.md)) — `list-backends` shows them as `world-grounded`, and
`extract --backend gvhmr` redirects to the run-tool-then-`import-hmr` workflow. Running their
inference inside RobotDance (rather than redirecting) remains future work.

## 2. Retargeting (human / SMPL → robot joint angles)

- **GMR — General Motion Retargeting** (ICRA 2026, MIT licensed) — the closest OSS to
  RobotDance's retarget. 18 robots incl. Unitree G1/H1/H1-2 and **Booster T1/K1**, real-time on
  CPU (60–70 fps), IK on `mink` + MuJoCo, inputs SMPL/BVH/FBX/**GVHMR-video**/VR. It is the
  retargeter for TWIST. <https://github.com/YanjieZe/GMR>
- **G-DReaM** (graph-conditioned diffusion retargeting across embodiments) and neural retargeting
  ("Make Tracking Easy") — learned alternatives to per-joint IK.

**Relevance:** GMR overlaps RobotDance's `retarget` / `retarget-ik` and targets the same robots.
As of v0.95 it is a **registered `external` retarget backend** (`robotdance list-retargeters`
shows `kinematic` / `actuator-ik` builtin alongside `gmr`); the open part is wiring GMR's
`mink`-based IK as a runnable backend and benchmarking it against RobotDance's actuator-space IK
on shared clips.

## 3. Whole-body control / imitation / teleoperation (RL policies, sim→real on hardware)

- **H2O** and **OmniH2O** — RGB-camera real-time human→humanoid teleop; align SMPL to the robot,
  then **remove infeasible motions with a trained privileged imitation policy**.
  <https://human2humanoid.com/>
- **HumanPlus**, **ExBody / ExBody2**, **HOVER**, **GMT**, **TWIST / TWIST2**, **UniTracker**,
  **AMO**, **KungfuBot** — expressive / dynamic whole-body tracking executed on real robots.

**Relevance:** this is the execution layer beyond RobotDance's scope today. RobotDance has RL
tracking + a ROS2 safety-gated runtime, but its centre of gravity is the *compiler + dataset +
certificate*, not a hardware controller. These are downstream consumers of clean RD-MIR.

## 4. Physics-based imitation in simulation (feasibility)

- **PHC / PHC-MJX / UHC / PULSE** (Luo et al.) — SMPL avatars driven in Isaac/MuJoCo; high-fidelity
  imitation with fail-state recovery. <https://github.com/ZhengyiLuo/PHC>

**Relevance:** these *learn* a controller to test feasibility; RobotDance instead emits an
**analytic certificate** (torque / balance / airborne from real URDF inertia) that is cheap and
interpretable. Different trade-off: a policy is more permissive but opaque; the certificate is
conservative but diagnostic.

## 5. Datasets + feasibility curation (RobotDance's closest neighbours)

- **PHUMA** (2025) — physics-aware curation that corrects foot-ground contact and filters severe
  artifacts. <https://arxiv.org/abs/2510.26236>
- **OpenT2M** (2026) — 1M+ sequences with physical-aware quality control.
- **openhe/g1-retargeted-motions** — 174 SMPL→G1 sequences via Mink, on Hugging Face.
- **AMASS / HumanML3D / Motion-X / BABEL** — source corpora RobotDance already imports
  (`import-humanml3d`, `import-babel`, `import-motionx`).

**Relevance:** these are the most direct positioning peers. They filter infeasible motion via
rule thresholds (PHUMA) or learned policies (H2O); RobotDance filters via the analytic
certificate and adds license/provenance discipline on top.

## What RobotDance does differently

1. **Feasibility as a transparent gate.** An analytic MuJoCo certificate (torque/balance/airborne
   with real URDF inertia) instead of a learned policy (H2O) or hand-tuned rules (PHUMA) —
   interpretable and diagnostic by design. The honest cost: it's conservative (monocular squats
   REJECT on depth-limited balance, shown not hidden).
2. **License-safety / provenance.** Manifest-driven, never bundles raw video/mocap/weights/meshes
   — only renders, numeric constants, and attribution. Under-emphasized across the field.
3. **Compiler framing.** One repo: RD-MIR standard IR → multi-embodiment retarget (G1/H1 /
   Booster T1 / Apollo) → certificate → embeddings/retrieval → benchmark → ROS2 safety runtime.
4. **Honesty about monocular limits.** `motion-doctor` flags mirror / depth-collapse / foot-skate
   / multi-subject per clip; limits are surfaced, not papered over.

## Gaps to close (borrow from the neighbours)

- **World-grounded extraction backend** (GVHMR / WHAM / TRAM) to relax the depth-limited frontier
  — done as registered `import`-mode backends (v0.94); the open part is running their inference
  in-process instead of redirecting to `import-hmr`.
- **GMR as an optional retarget backend** — registered in the retarget registry (v0.95); the open
  part is making it runnable (wire its `mink` IK) and a shared-clip benchmark vs actuator-space IK.
- **Certificate vs. learned-filter study** — compare the analytic certificate's REJECT set
  against H2O's privileged-policy filtering and PHUMA's rule thresholds.
- **Curation interop** — align RD-MIR quality fields with PHUMA / OpenT2M conventions.
