# RobotDance — Plan (HumanoidBattle pivot)

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
- Plan: keep making the battle more fun *and* launch with it as the hook. Don't rename the repo
  yet (breaks links, no traffic upside) — decided 2026-06-08.

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

- [ ] **Physical tournament** — run the whole bracket as `demo-fight` bouts (boxing), not kinematic
      kata scoring. Crown a *fighting* champion. Round-robin or single-elim, hit-count standings.
- [ ] **Mesh fights** — render the bout with real robot meshes (G1/H1/H2 URDFs we already have)
      instead of capsules, for a far more impressive GIF. Reuse `render_real_video_gif` mesh path.
- [ ] **Special moves from real video** — add the karate kata / kathak clips as selectable "moves"
      so a fighter can throw a real extracted technique. Ties the video pipeline into the game.
- [ ] **Leaderboard / ranking** — persist results to `LEADERBOARD.md`, ELO across matchups,
      "hall of champions". Makes outcomes feel like a sport.
- [ ] **More moves + balance** — expand the move roster (hooks, kicks, dodges), tune difficulty so
      outcomes spread (avoid 10–10 draws); per-body strengths (tall = reach, compact = precision).

## Priority 3 — Tech frontier that deepens the game

The honest unsolved core; better here → a *real* fight instead of choreography:

- [ ] **Balance controller** so a humanoid can track motion under forward dynamics without falling
      (pinned-base → assisted → free). This is the gate to **true contact sparring** (punches that
      land with reaction, stumbles, KOs). Likely RL (the `tracking_env` is the seed).
- [ ] **Depth frontier** (continues `--stabilize-depth` / `--balance-refine`): observability-weighted
      depth, contact-aware retarget — so extracted real-video moves are physically feasible enough to
      fight with.

## Non-goals / decisions

- **No repo rename** to "HumanoidBattle" yet — breaks Colab/install/badge links, discards the
  RobotDance identity, and brings no traffic on its own. Revisit *after* a launch lands.
- **No fake physics.** Hits stay geometric and clearly labeled until a real balance controller makes
  contact dynamics stable. No gimmicks that pretend the fight is fully simulated when it isn't.
- Keep license-safety: only renders + numeric motion, never raw video/mocap/meshes.

## Workflow (unchanged)

One feature per increment → `ruff` + full `pytest` → bump version (pyproject + CITATION + CHANGELOG)
→ commit → push → CI green → prerelease. Skeleton/scoring demos stay URDF/GPU-free (run in Colab);
MuJoCo rendering needs EGL + the `sim` extra and is owner/local-only.
