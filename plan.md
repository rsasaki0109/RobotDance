# HumanoidBattle — Plan

> Living plan. The original research roadmap is in [`docs/ROADMAP.md`](docs/ROADMAP.md); this file
> tracks the **current strategic direction**: turn the motion-compiler core into a fun, shareable
> **HumanoidBattle** and actually get it in front of people. Updated as of v0.138.

## TL;DR

- The **tech is done enough**: monocular video → RD-MIR → retarget to 6 real humanoids → MuJoCo
  feasibility gate → benchmarks. README, Colab, demos are polished.
- **Stars are not growing because nobody has seen it.** Traffic (14d): ~7 unique visitors, all
  internal, **zero external referrers**. This is a distribution problem, not a quality problem.
- **Two levers, in priority order:**
  1. **Distribution** — launch it (Show HN / r/robotics / X). Only the owner can press this button.
  2. **A viral hook** — **HumanoidBattle**: humanoids that fight/compete. Fun things get shared.
- Plan: keep making the battle more fun *and* launch with it as the hook. **Repo renamed to
  HumanoidBattle** (2026-06-08); pip/CLI package stays `robotdance`. GitHub redirects old URLs.

## Where we are (v0.138)

Done this arc:
- ⚔️ `demo-battle` — 1v1 kata face-off, scored purely from real metrics (reach / bone-dir /
  foot-sliding / ROM, optional MuJoCo balance/torque). Transparent, no random numbers.
- 🏆 `demo-tournament` — single-elim bracket, best-of-3 with **move difficulty + feasibility whiff**
  (hard moves score more *if your body can land them*). Champion: **Fourier N1** (most human-like
  proportions → lowest reach error → cleanest execution).
- 🥊 `demo-fight` — **two humanoids boxing in one MuJoCo scene** (MjSpec.attach, red/blue corners,
  lights/shadows), live hit counter HUD. Honest scope: kinematic playback (no fall) + geometric hit
  detection (not contact forces). G1 vs H1 → H1 wins 10–6 on reach, G1 answers with body shots.
- Star-support assets: multi-embodiment hero, kathak 2nd demo, Colab quickstart, `--stabilize-depth`
  before/after, launch kit drafts (`/home/sasaki/tmp/LAUNCH.md`), social preview image.

## Priority 1 — Distribution (the real bottleneck)

This is what actually moves stars. Owner-action required; everything below removes friction.

- [ ] **Set the GitHub social preview** image (Settings → General → Social preview) so shared links
      show the multi-embodiment hero / fight GIF, not grey text. (Asset ready; API can't set it.)
- [ ] **Post once, with a GIF.** Best hook now is the 🥊 boxing fight or the 🏆 tournament.
      Drafts ready in the launch kit: Show HN, r/robotics + r/humanoid, an X thread.
      Timing: Tue–Thu US morning. Humanoids are hot (Unitree / Figure / Optimus).
- [ ] **Awesome-list PRs** (passive discovery): awesome-humanoid-robots, awesome-robotics,
      awesome-motion-capture.
- [ ] Reply to the first wave of comments (the owner; I can draft responses).

> One front-page hit > months of README polish. After traffic exists, polish converts it.

## Priority 2 — Make HumanoidBattle more fun (the hook)

Each is a self-contained increment (implement → test → release), most fun first:

- [x] **Physical tournament** — `demo-tournament --physical` runs single-elim bracket as
      `demo-fight` bouts (hit-count); `--moves boxing/karate/kathak` for best-of-N styles.
      Crown a *fighting* champion; final bout rendered as fight GIF.
- [x] **Mesh fights** — `demo-fight --mesh` renders with real G1/H1/H2 URDF meshes (pybullet)
      instead of MuJoCo capsules; shared `robotdance_sim/mesh_render.py` with `render_real_video_gif`.
- [x] **Special moves from real video** — karate kata / kathak clips as selectable moves
      (`robot:motion` in battle/tournament; `demo-fight --style karate|kathak`). Bundled RD-MIR
      fixtures (numeric motion only, CC BY-SA attribution in source_ref).
- [x] **Leaderboard / ranking** — `demo-tournament --record` persists ELO + bout log +
      Hall of Champions to `docs/benchmark/HUMANOID_BATTLE_LEADERBOARD.md`
      (`humanoid_battle_state.json`). Physical = ELO; kata = hall only.
- [x] **More moves + balance** — `hook`/`kick`/`dodge` fight styles + kinematic moves; per-height
      reach/precision hit radii; draw tiebreak (body hits → height). `robotdance_sim/fight_moves.py`.

## Priority 3 — Tech frontier that deepens the game

The honest unsolved core; better here → a *real* fight instead of choreography:

- [~] **Balance controller** so a humanoid can track motion under forward dynamics without falling
      (pinned-base → assisted → free). **v0.144**: `demo-assisted` — single-robot PD-only rollout via
      `TrackingEnv` (`robotdance_sim/assisted_playback.py`). **v0.145**: `benchmark-assisted` で
      fight × robot の survival を raw/refine 比較。**v0.146**: fight motion RL tracking CLI。
      **v0.147**: `benchmark-assisted --rl` で PD 失敗組に RL 列を追加。
      **v0.148**: `demo-fight --assisted` — 1 体 PD-only 物理追従、相手 kinematic。
      **v0.149**: `--assisted --rl` — 同枠で PPO tracking 追従（`rollout_rl`）。
      **v0.150**: physical tournament 決勝 assisted/RL + benchmark rescued-by-RL-only。
      **v0.151**: `--assisted champion`（省略時も champion）— 決勝 GIF でチャンピオン側を自動物理追従。
      **v0.155**: 決勝 GIF HUD に `assisted_survival`（PD/RL %）を焼き込み（`_fight_hud`）。
      **v0.158**: `demo-fight --sparring` — 2 体同時 PD 物理（limb 接触、幾何採点は維持）。
      Full contact-dynamics scoring still open.
- [~] **Depth frontier** (continues `--stabilize-depth` / `--balance-refine`): **v0.144**:
      `refine_for_fight()` + `demo-fight --depth-refine` / `demo-assisted --depth-refine` wire
      stabilize + balance into the fight pipeline. **v0.145**: assisted survival benchmark で効果を
      可視化（`docs/benchmark/ASSISTED_SURVIVAL.md`）。Contact-aware retarget still open.
- [x] **GMR retarget backend** — **v0.153**: `retarget --backend gmr` が GMR mink IK を実行
      （`robotdance_retarget/gmr_backend.py`、clone + `pip install -e GMR/` 要）。対応 robot:
      G1/H1/H2/T1/N1。**v0.154**: `benchmark-assisted --retarget-backend kinematic gmr` で
      survival を backend 別比較（`rescued_by_gmr` サマリ）。**v0.156**: `demo-fight` /
      `demo-assisted` / トーナメント決勝に `--retarget-backend gmr`。
- [x] **GVHMR in-process 抽出** — **v0.157**: `extract --backend gvhmr` が GVHMR 推論を
      in-process 実行（`gvhmr_backend.py`、clone + ckpt + CUDA 要）。WHAM は import-hmr のまま。

## Non-goals / decisions

- **Repo is HumanoidBattle**; Python package / CLI remain `robotdance` (PyPI・import 互換).
  Old `RobotDance` GitHub URLs redirect automatically.
- **No fake physics.** Hits stay geometric and clearly labeled until a real balance controller makes
  contact dynamics stable. No gimmicks that pretend the fight is fully simulated when it isn't.
- Keep license-safety: only renders + numeric motion, never raw video/mocap/meshes.

## Workflow (unchanged)

One feature per increment → `ruff` + full `pytest` → bump version (pyproject + CITATION + CHANGELOG)
→ commit → push → CI green → prerelease. Skeleton/scoring demos stay URDF/GPU-free (run in Colab);
MuJoCo rendering needs EGL + the `sim` extra and is owner/local-only.
