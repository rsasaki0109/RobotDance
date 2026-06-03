"""robotdance core CLI.

サブコマンド:
  validate  RD-Manifest / RD-MIR / RD-Embodiment の JSON を specs/ の JSON Schema で検証
  synth     合成ダンスモーション RD-MIR を生成（pose モデル不要のデモ種データ）
  view      RD-MIR の 3D スケルトンを GIF に描画

pose 抽出・retarget・sim は後続フェーズで追加する。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# specs/ はリポジトリルート直下（このファイルから 2 つ上）に固定配置されている。
_SPECS_DIR = Path(__file__).resolve().parent.parent / "specs"

_SCHEMAS = {
    "manifest": _SPECS_DIR / "rd-manifest" / "rd-manifest.schema.json",
    "mir": _SPECS_DIR / "rd-mir" / "rd-mir.schema.json",
    "embodiment": _SPECS_DIR / "rd-embodiment" / "rd-embodiment.schema.json",
}


def _validate(spec: str, path: Path) -> int:
    try:
        import jsonschema  # 遅延 import: validate 以外では不要
    except ImportError:
        print("error: jsonschema が必要です（pip install jsonschema）", file=sys.stderr)
        return 2

    schema = json.loads(_SCHEMAS[spec].read_text(encoding="utf-8"))
    instance = json.loads(path.read_text(encoding="utf-8"))
    validator = jsonschema.Draft202012Validator(schema)
    errors = sorted(validator.iter_errors(instance), key=lambda e: e.path)
    if errors:
        print(f"✗ {path} は rd-{spec} v0 schema に違反しています:")
        for err in errors:
            loc = "/".join(str(p) for p in err.path) or "(root)"
            print(f"  - {loc}: {err.message}")
        return 1
    print(f"✓ {path} は rd-{spec} v0 schema に適合しています")
    return 0


def _synth(out: Path, duration: float, fps: float) -> int:
    from .synthetic import generate_dance

    mir = generate_dance(duration=duration, fps=fps)
    mir.save(out)
    print(f"✓ 合成 RD-MIR を書き出しました: {out} "
          f"({mir.num_frames} frames, {mir.fps:g} fps, {mir.duration:g}s)")
    return 0


def _view(path: Path, out: Path, stride: int) -> int:
    from .rd_mir import RdMir
    from robotdance_viewer.skeleton_view import render_gif

    mir = RdMir.load(path)
    render_gif(mir, out, stride=stride)
    print(f"✓ スケルトン GIF を書き出しました: {out}")
    return 0


def _retarget(path: Path, out: Path, robot: str) -> int:
    from .rd_mir import RdMir
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    mir = RdMir.load(path)
    motion = retarget(mir, get_morphology(robot))
    motion.save(out)
    m = motion.retarget_metrics or {}
    print(f"✓ {robot} RD-Motion を書き出しました: {out}")
    print(f"  height_scale={m.get('height_scale')} "
          f"bone_direction_cosine={m.get('bone_direction_cosine')} "
          f"foot_sliding={m.get('foot_sliding_m_per_frame')}")
    print("  ⚠️ kinematic preview のみ — 物理 sim 未検証（Phase 2）")
    return 0


def _view_pair(human_path: Path, robot_path: Path, out: Path, stride: int) -> int:
    from .rd_mir import RdMir
    from .rd_motion import RdMotion
    from robotdance_viewer.skeleton_view import render_side_by_side

    mir = RdMir.load(human_path)
    motion = RdMotion.load(robot_path)
    render_side_by_side(
        [
            (mir.keypoints_3d_array(), "Human (RD-MIR)", "#1f77b4"),
            (motion.keypoints_3d_array(), f"{motion.robot_name} (RD-Motion)", "#ff7f0e"),
        ],
        out,
        fps=mir.fps,
        stride=stride,
    )
    print(f"✓ side-by-side GIF を書き出しました: {out}")
    return 0


def _demo_g1(out: Path, stride: int) -> int:
    """synth → retarget → side-by-side を一括実行する便利コマンド。"""
    from .synthetic import generate_dance
    from robotdance_retarget.kinematic import retarget_to_g1
    from robotdance_viewer.skeleton_view import render_side_by_side

    mir = generate_dance()
    motion = retarget_to_g1(mir)
    render_side_by_side(
        [
            (mir.keypoints_3d_array(), "Human (RD-MIR)", "#1f77b4"),
            (motion.keypoints_3d_array(), f"{motion.robot_name} (RD-Motion)", "#ff7f0e"),
        ],
        out,
        fps=mir.fps,
        stride=stride,
    )
    m = motion.retarget_metrics or {}
    print(f"✓ G1 side-by-side デモ GIF を書き出しました: {out}")
    print(f"  height_scale={m.get('height_scale')} foot_sliding={m.get('foot_sliding_m_per_frame')}")
    return 0


def _import_hmr(path: Path, source: str, fps: float | None, out: Path) -> int:
    """HMR（4DHumans/GVHMR）の SMPL 出力（.npz 交換フォーマット）を RD-MIR 化する（§4.1）。"""
    from robotdance_perception.hmr import load_hmr_npz

    mir = load_hmr_npz(path, source=source, fps=fps)
    mir.save(out)
    q = mir.quality_metrics or {}
    print(f"✓ HMR({source}) {path.name} → RD-MIR: {out}")
    print(f"  frames={mir.num_frames} fps={mir.fps:g} extractor={q.get('extractor')} "
          f"jitter={q.get('jitter_after')} license_state={mir.license_state}")
    print("  ⚠️ skeleton-first（近似 rest offset・betas 未使用）。HMR 推論はツール側、本 adapter は"
          " SMPL→canonical 変換。in-the-wild 由来は license_state=unknown（公開前に権利確認）。")
    return 0


def _extract(video: Path, out: Path, model: Path | None) -> int:
    from robotdance_perception.mediapipe_adapter import extract_motion

    mir = extract_motion(video, model_path=model)
    mir.save(out)
    print(f"✓ {video.name} → RD-MIR: {out}")
    print(f"  frames={mir.num_frames} fps={mir.fps:g} "
          f"mean_confidence={(mir.quality_metrics or {}).get('mean_confidence')} "
          f"license_state={mir.license_state}")
    return 0


def _video_to_robot(video: Path, robot: str, out: Path, stride: int) -> int:
    """local 動画 → RD-MIR → retarget → MuJoCo 検証 → human|robot side-by-side（Shorts to humanoid）。"""
    from robotdance_perception.mediapipe_adapter import extract_motion
    from robotdance_retarget.kinematic import retarget
    from robotdance_sim.mujoco_backend import certify
    from robotdance_unitree import get_morphology
    from robotdance_viewer.skeleton_view import render_side_by_side

    morph = get_morphology(robot)
    mir = extract_motion(video)
    motion = retarget(mir, morph)
    certify(motion, morph)
    cert = motion.sim_certificate
    print(f"  extracted {mir.num_frames} frames → {robot}: {cert['verdict']} {cert['reasons']}")
    render_side_by_side(
        [
            (mir.keypoints_3d_array(), f"video: {video.stem}", "#1f77b4"),
            (motion.keypoints_3d_array(), robot, "#ff7f0e"),
        ],
        out, fps=mir.fps, stride=stride,
        verdicts=[("SOURCE", "#1f77b4"), (cert["verdict"], "#2ca02c" if cert["passed"] else "#d62728")],
    )
    print(f"✓ Shorts-to-humanoid GIF を書き出しました: {out}")
    return 0


def _serve(path: Path, speed: float, ros2: bool, allow_uncertified: bool) -> int:
    from .rd_motion import RdMotion
    from robotdance_ros2.motion_server import MotionServer
    from robotdance_ros2.safety_guard import SafetyGuard, SafetyLimits

    if ros2:
        from robotdance_ros2.motion_server_node import main as node_main

        argv = [str(path), "--speed", str(speed)]
        if allow_uncertified:
            argv.append("--allow-uncertified")
        return node_main(argv)

    # dry-run（ROS2 不要）: 安全ゲートとフレーム整形をシミュレートする。
    motion = RdMotion.load(path)
    guard = SafetyGuard(SafetyLimits(require_certificate=not allow_uncertified), speed_scale=speed)
    server = MotionServer(motion, guard)
    pre = server.precheck()
    print(f"precheck: {pre.status.value} {pre.reasons}")
    frames = server.export_frames()
    warns = sum(1 for _, s in frames if s.status.value == "WARNING")
    aborted = any(s.is_abort for _, s in frames)
    print(f"streamed {len(frames)} frames (speed×{speed}) warnings={warns} aborted={aborted}")
    if not frames:
        print("  ⛔ 再生されませんでした（safety guard が遮断）")
    return 0


def _demo_runtime() -> int:
    """安全ゲートの実演: certified(PASS) は再生、uncertified/REJECT は遮断。"""
    from .synthetic import generate_backflip, generate_dance
    from robotdance_retarget.kinematic import retarget
    from robotdance_ros2.motion_server import MotionServer
    from robotdance_ros2.safety_guard import SafetyGuard
    from robotdance_unitree import get_morphology

    morph = get_morphology("unitree_g1")
    try:
        from robotdance_sim.mujoco_backend import certify
    except ImportError:
        print("mujoco 無し: demo-runtime は sim_certificate を要するためスキップ")
        return 0

    for label, mir in [("dance(PASS見込み)", generate_dance()),
                       ("backflip(REJECT見込み)", generate_backflip())]:
        motion = certify(retarget(mir, morph), morph)
        server = MotionServer(motion, SafetyGuard())
        frames = server.export_frames()
        verdict = (motion.sim_certificate or {}).get("verdict")
        gate = "再生" if frames else "遮断（ABORT）"
        print(f"  {label:22s} cert={verdict:6s} → motion_server: {gate}（{len(frames)} frames）")
    print("  → safety guard は certificate REJECT を実機/再生前に遮断する（§5.6）")
    return 0


def _benchmark(robots: list[str], motions_dir: Path | None, with_sim: bool, out_dir: Path) -> int:
    from robotdance_benchmarks.report import aggregate_by_robot, write_csv, write_markdown
    from robotdance_benchmarks.suite import default_motion_suite, run_benchmark, run_from_dir

    if motions_dir is not None:
        report = run_from_dir(motions_dir, robots, with_sim=with_sim)
    else:
        report = run_benchmark(default_motion_suite(), robots, with_sim=with_sim)
    csv_path = write_csv(report, out_dir / "benchmark.csv")
    md_path = write_markdown(report, out_dir / "LEADERBOARD.md")
    print(f"✓ benchmark: {len(report['rows'])} runs "
          f"(sim {'on' if report['sim_available'] else 'off'})")
    print(f"  {csv_path}\n  {md_path}")
    for a in aggregate_by_robot(report):
        print(f"  {a['robot']:12s} PASS率={a['pass_rate']} "
              f"bone_cos={a['mean_bone_dir_cos']} foot_sliding={a['mean_foot_sliding']}")
    return 0


def _retarget_ik(path: Path, urdf: Path, out: Path, steps: int) -> int:
    from .rd_mir import RdMir
    from robotdance_retarget.actuator_ik import actuator_retarget

    motion = actuator_retarget(RdMir.load(path), urdf, steps=steps)
    motion.save(out)
    m = motion.retarget_metrics or {}
    jr = motion.joint_rotations or {}
    print(f"✓ actuator-space IK → {out}")
    print(f"  {m.get('actuated_joints')} 関節角を出力（{len(jr.get('angles_rad', []))} frames）")
    print(f"  IK 位置誤差 mean={m.get('ik_mean_pos_error_m')}m max={m.get('ik_max_pos_error_m')}m")
    print("  ⚠️ 参照 IK（位置合わせ）。バランス policy ではない（sim_certificate で別途検証）。")
    return 0


def _import_urdf(urdf: Path, name: str, save: Path | None) -> int:
    import json

    import jsonschema

    from robotdance_unitree.urdf_import import urdf_to_morphology

    morph = urdf_to_morphology(urdf, name=name)
    emb = morph.to_rd_embodiment()
    schema = json.loads(
        (_SPECS_DIR / "rd-embodiment" / "rd-embodiment.schema.json").read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema).validate(emb)
    print(f"✓ URDF → RobotMorphology: {name}")
    print(f"  nominal_height={morph.nominal_height:.3f} m  joints={len(emb['joint_names'])}")
    print("  ⚠️ 寸法は実 URDF 由来。torso 連鎖・toe は合成、質量は近似（v0）。")
    if save is not None:
        save.write_text(json.dumps(emb, indent=2), encoding="utf-8")
        print(f"  → RD-Embodiment 保存: {save}")
    return 0


def _train_encoder(out: Path, epochs: int, device: str | None) -> int:
    from robotdance_models.train import train_encoder

    res = train_encoder(out_path=out, epochs=epochs, device=device)
    h = res["loss_history"]
    print(f"✓ motion encoder 学習完了: {out}")
    print(f"  windows={res['windows']} device={res['device']} epochs={epochs}")
    print(f"  masked 再構成 loss: {h[0]:.4f} → {h[-1]:.4f}（{100 * (1 - h[-1] / max(h[0], 1e-9)):.0f}% 減少）")
    return 0


def _train_text_motion(out: Path, epochs: int, device: str | None) -> int:
    from robotdance_models.contrastive import train_text_motion

    res = train_text_motion(out_path=out, epochs=epochs, device=device)
    h = res["loss_history"]
    print(f"✓ text-motion contrastive 学習完了: {out}")
    print(f"  pairs={res['pairs']} motions={res['motions']} device={res['device']} epochs={epochs}")
    print(f"  InfoNCE loss: {h[0]:.4f} → {h[-1]:.4f}")
    print(f"  caption→motion retrieval: group top-1 {100 * res['group_top1']:.0f}% "
          f"/ exact top-1 {100 * res['train_top1']:.0f}%")
    return 0


def _train_tokenizer(out: Path, epochs: int, codes: int, device: str | None) -> int:
    from robotdance_models.tokenizer import train_tokenizer

    res = train_tokenizer(out_path=out, epochs=epochs, num_codes=codes, device=device)
    h = res["loss_history"]
    print(f"✓ motion VQ-VAE 学習完了: {out}")
    print(f"  windows={res['windows']} device={res['device']} epochs={epochs}")
    print(f"  loss: {h[0]:.4f} → {h[-1]:.4f}  /  再構成 MSE(正規化): {res['recon_mse']:.5f}")
    print(f"  codebook 使用率: {res['codes_used']}/{res['num_codes']} "
          f"({100 * res['codebook_usage']:.0f}%)  /  tokens/window: {res['tokens_per_window']}")
    return 0


def _demo_tokenizer(out: Path, checkpoint: Path | None, epochs: int, stride: int) -> int:
    """モーションを離散トークン化し、圧縮率・再構成・codebook を実演する（§4.2）。"""
    import numpy as np

    from robotdance_core.synthetic import generate_backflip, generate_dance
    from robotdance_models.tokenizer import MotionTokenizer, train_tokenizer

    ckpt = checkpoint
    if ckpt is None:
        ckpt = out.with_suffix(".pt")
        train_tokenizer(out_path=ckpt, epochs=epochs)
    tok = MotionTokenizer(ckpt)

    suite = {
        "dance_1.0": generate_dance(beats_per_second=1.0),
        "dance_1.6": generate_dance(beats_per_second=1.6),
        "backflip": generate_backflip(),
    }
    print("🔤 motion → 離散トークン（VQ-VAE）:")
    for name, m in suite.items():
        ids = tok.encode(m)
        orig, rec = tok.reconstruct(m)
        rmse = float(np.sqrt(((orig - rec) ** 2).mean()))
        frames = orig.shape[0]  # タイルでカバーされる実フレーム数
        ratio = frames / max(len(ids), 1)
        print(f"  {name:10s} {frames:3d}f → {len(ids):2d} tokens（{ratio:.0f}× 圧縮, "
              f"uniq={len(set(ids.tolist()))}） 再構成 RMSE={rmse:.4f}")

    # dance を元 vs トークン再構成で side-by-side GIF 化。
    orig, rec = tok.reconstruct(suite["dance_1.0"])
    from robotdance_viewer.skeleton_view import render_side_by_side

    render_side_by_side(
        [(orig, "original", "#1f77b4"), (rec, "VQ reconstruction", "#d62728")],
        out, stride=stride,
        verdicts=[("source motion", "#1f77b4"), ("from discrete tokens", "#d62728")],
    )
    print(f"✓ 再構成デモ GIF: {out}")
    return 0


def _train_text2motion(tokenizer: Path, out: Path, epochs: int, device: str | None) -> int:
    from robotdance_models.text2motion import train_text2motion

    res = train_text2motion(tokenizer_ckpt=tokenizer, out_path=out, epochs=epochs, device=device)
    h = res["loss_history"]
    print(f"✓ text-conditioned 生成 prior 学習完了: {out}")
    print(f"  sequences={res['sequences']} vocab={res['vocab']} device={res['device']} epochs={epochs}")
    print(f"  next-token loss: {h[0]:.4f} → {h[-1]:.4f}  /  精度: {100 * res['next_token_acc']:.0f}%")
    return 0


def _generate_text(caption: str, checkpoint: Path, out: Path, gif: Path | None,
                   temperature: float, seed: int, stride: int) -> int:
    """caption からモーションを生成し RD-MIR を保存する（§4.2 text→motion 生成）。"""
    from robotdance_models.text2motion import TextToMotion

    g = TextToMotion(checkpoint)
    mir = g.generate(caption, temperature=temperature, seed=seed)
    mir.save(out)
    kp = mir.keypoints_3d_array()
    energy = float(kp.std(axis=0).mean())
    print(f'🎬 "{caption}" → 生成モーション')
    print(f"  RD-MIR: {out}  frames={mir.num_frames}  運動量(energy)={energy:.3f}")
    if gif is not None:
        from robotdance_viewer.skeleton_view import render_side_by_side

        render_side_by_side([(kp, caption, "#9467bd")], gif, stride=stride,
                            verdicts=[("text → motion", "#9467bd")])
        print(f"  GIF: {gif}")
    print("  ⚠️ 生成物は物理的に妥当とは限らない — retarget → sim_certificate（validate-sim）で必ず検証する。")
    return 0


def _train_prior(tokenizer: Path, out: Path, epochs: int, device: str | None) -> int:
    from robotdance_models.prior import train_prior

    res = train_prior(tokenizer_ckpt=tokenizer, out_path=out, epochs=epochs, device=device)
    h = res["loss_history"]
    print(f"✓ motion token prior 学習完了: {out}")
    print(f"  sequences={res['sequences']} vocab={res['vocab']} device={res['device']} epochs={epochs}")
    print(f"  next-token loss: {h[0]:.4f} → {h[-1]:.4f}  /  精度: {100 * res['next_token_acc']:.0f}%")
    return 0


def _demo_generate(out: Path, prior_ckpt: Path | None, epochs: int, temperature: float,
                   stride: int) -> int:
    """token prior でモーションを生成・補完する（§4.2）。"""
    import numpy as np

    from robotdance_core.synthetic import generate_dance
    from robotdance_models.prior import MotionGenerator, train_prior
    from robotdance_models.tokenizer import train_tokenizer
    from robotdance_viewer.skeleton_view import render_side_by_side

    pri = prior_ckpt
    if pri is None:
        tok = out.with_name("demo_tokenizer.pt")
        pri = out.with_suffix(".pt")
        train_tokenizer(out_path=tok, epochs=epochs)
        train_prior(tokenizer_ckpt=tok, out_path=pri, epochs=2 * epochs)
    gen = MotionGenerator(pri)

    print("🎲 token prior による生成（BOS から自己回帰サンプリング）:")
    panels = []
    colors = ["#9467bd", "#2ca02c", "#e377c2"]
    for i, sd in enumerate((0, 1, 2)):
        m = gen.generate(length=16, temperature=temperature, seed=sd)
        kp = m.keypoints_3d_array()
        jit = float(np.linalg.norm(np.diff(kp, n=2, axis=0), axis=2).mean())
        print(f"  sample seed={sd}: frames={m.num_frames} jitter={jit:.4f}")
        panels.append((kp, f"generated #{sd}", colors[i]))
    render_side_by_side(panels, out, stride=stride,
                        verdicts=[("from token prior", "#9467bd")] * len(panels))
    print(f"✓ 生成デモ GIF: {out}")

    # 補完: 元 dance の先頭を残して続きを生成。
    dance = generate_dance(beats_per_second=1.0)
    comp, toks = gen.complete(dance, keep=4, temperature=temperature, seed=0)
    print(f"  補完: 先頭 4 トークンを残し続きを生成 → {len(toks)} tokens / {comp.num_frames} frames")
    print("  ⚠️ 生成物は物理的に妥当とは限らない — retarget → sim_certificate（validate-sim）で必ず検証する。")
    return 0


def _train_tracking(out: Path, robot: str, iterations: int, device: str | None,
                    suite: bool) -> int:
    """RL tracking policy（§4.5）を学習する。--suite で複数運動を 1 方策に汎化。"""
    from robotdance_core.synthetic import generate_dance
    from robotdance_models.tracking_policy import (
        train_multi_tracking_policy,
        train_tracking_policy,
    )
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    morph = get_morphology(robot)
    if suite:
        items = _tracking_suite(morph)
        refs = [r for _, r in items]
        policy, info = train_multi_tracking_policy(refs, morph, iterations=iterations,
                                                   device=device, out_path=out)
        rh = info["return_history"]
        print(f"✓ multi-motion RL tracking policy 学習完了: {out}")
        print(f"  robot={robot} device={info['device']} iterations={iterations} "
              f"refs={info['num_references']} obs={info['obs_dim']} act={info['action_dim']}（base 非駆動）")
        print(f"  episode return: {rh[0]:.2f} → {rh[-1]:.2f}（PPO, round-robin）")
        for i, (name, _r) in enumerate(items):
            m = policy.rollout(i)[1]
            print(f"  {name:7s}: 生存 {m['survived_frames']}/{m['reference_frames']} "
                  f"(survival {m['survival_ratio']:.0%}) / pose RMSE {m['mean_pose_rmse']:.3f}")
        print("  ⚠️ v0 baseline: 1 方策が運動に応じ追従。PD 超え精度・摂動/実機転移は今後。")
        return 0

    ref = retarget(generate_dance(duration=2.0, arm_amp=0.6, sway_amp=0.08), morph)
    policy, info = train_tracking_policy(ref, morph, iterations=iterations,
                                         device=device, out_path=out)
    rh = info["return_history"]
    m = policy.rollout()[1]
    print(f"✓ RL tracking policy 学習完了: {out}")
    print(f"  robot={robot} device={info['device']} iterations={iterations} "
          f"obs={info['obs_dim']} act={info['action_dim']}（base 非駆動）")
    print(f"  episode return: {rh[0]:.2f} → {rh[-1]:.2f}（PPO）")
    print(f"  rollout: 生存 {m['survived_frames']}/{m['reference_frames']} frames "
          f"(survival {m['survival_ratio']:.0%}) / pose RMSE {m['mean_pose_rmse']:.3f}")
    print("  ⚠️ v0 baseline: 関節 PD への残差を学習。近似質量・単一参照で完全追従ではない。")
    return 0


def _demo_track(out: Path, robot: str, iterations: int, stride: int) -> int:
    """参照運動を RL policy で物理追従し、参照 vs 追従を side-by-side 描画する（§4.5）。"""
    from robotdance_core.synthetic import generate_dance
    from robotdance_models.tracking_policy import train_tracking_policy
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology
    from robotdance_viewer.skeleton_view import render_side_by_side

    morph = get_morphology(robot)
    ref = retarget(generate_dance(duration=2.0, arm_amp=0.6, sway_amp=0.08), morph)
    print(f"🤸 RL tracking policy を学習中（{robot}, PPO {iterations} iters, base 非駆動）...")
    policy, info = train_tracking_policy(ref, morph, iterations=iterations)
    motion, m = policy.rollout()
    print(f"  episode return: {info['return_history'][0]:.2f} → {info['return_history'][-1]:.2f}")
    print(f"  物理ロールアウト: 生存 {m['survived_frames']}/{m['reference_frames']} "
          f"(survival {m['survival_ratio']:.0%}) / pose RMSE {m['mean_pose_rmse']:.3f}")

    ref_kp = ref.keypoints_3d_array()
    trk_kp = motion.keypoints_3d_array()
    n = min(len(ref_kp), len(trk_kp))
    panels = [
        (ref_kp[:n], "reference (kinematic)", "#1f77b4"),
        (trk_kp[:n], "RL tracked (physics)", "#d62728"),
    ]
    render_side_by_side(panels, out, stride=stride,
                        verdicts=[("reference", "#1f77b4"),
                                  (f"survival {m['survival_ratio']:.0%}", "#d62728")])
    print(f"✓ tracking デモ GIF: {out}")
    print("  ⚠️ v0 baseline: PD 残差を PPO で学習。倒れずに追従する足場で、SOTA tracking ではない。")
    return 0


def _demo_joint_safety() -> int:
    """関節空間 safety guard（§5.6）の位置/速度/加速度クランプを実演する。"""
    import numpy as np

    from robotdance_ros2.safety_guard import SafetyLimits, clamp_joint_trajectory

    # actuator-IK / tracking policy の関節角列に見立てた合成軌道（23-DOF, 30fps）。
    fps = 30.0
    dt = 1.0 / fps
    t = np.arange(60) / fps
    n = 23
    raw = 0.6 * np.sin(2 * np.pi * 0.8 * t)[:, None] * np.linspace(0.4, 1.0, n)[None, :]
    # 不安全な外れ値を注入。
    raw[:, 5] += np.linspace(0.0, 3.5, len(t))  # 緩やかに位置 limit(±2)超過 → 位置クランプ
    raw[20, 3] += 5.0                            # 単発スパイク → 速度クランプ
    raw[40, 7] += 3.0
    raw[41, 7] -= 3.0                            # 往復ジャーク → 加速度クランプ

    # 実機の関節 limit に見立てた位置範囲（±2.0 rad）。
    names = [f"joint_{i}" for i in range(n)]
    limits = SafetyLimits(
        max_joint_speed=12.0, max_joint_accel=200.0,
        joint_position_limits={nm: (-2.0, 2.0) for nm in names},
    )
    safe, rep = clamp_joint_trajectory(raw, dt, limits, names)

    print("🦾 関節空間 safety guard（実機コマンド直前の最終 gate, §5.6）")
    print(f"  関節数 {rep['joints']} / {rep['frames']} frames @ {fps:.0f}fps")
    print("  ── 速度（rad/s）──")
    print(f"    raw  max {rep['raw_max_joint_speed_rad_s']:8.2f}  →  "
          f"safe max {rep['safe_max_joint_speed_rad_s']:8.2f}  (limit {rep['max_joint_speed']:.0f})")
    print("  ── 加速度（rad/s²）──")
    print(f"    raw  max {rep['raw_max_joint_accel_rad_s2']:8.1f}  →  "
          f"safe max {rep['safe_max_joint_accel_rad_s2']:8.1f}  (limit {rep['max_joint_accel']:.0f})")
    print(f"  クランプ発生: 位置 {rep['position_limit_frames']} / 速度 "
          f"{rep['velocity_clamp_frames']} / 加速度 {rep['accel_clamp_frames']} frames")
    ok = (rep["safe_max_joint_speed_rad_s"] <= rep["max_joint_speed"] + 1e-6
          and float(np.abs(safe).max()) <= 2.0 + 1e-6)
    print(f"  ✓ 位置・速度を limit 内に整形: {ok}")
    print("  ⚠️ v0: 位置/速度は厳密 bound、加速度は best-effort。トルク/電流 limit は実機モデルで今後。")
    return 0


def _tracking_suite(morph):  # noqa: ANN001, ANN201
    """multi-motion tracking 用の参照スイート（gentle/normal/fast dance + idle）を作る。"""
    from robotdance_core.synthetic import generate_dance
    from robotdance_retarget.kinematic import retarget

    specs = [
        ("gentle", dict(arm_amp=0.6, sway_amp=0.08)),
        ("normal", dict(arm_amp=1.6, sway_amp=0.18)),
        ("fast", dict(beats_per_second=1.6, arm_amp=1.6, sway_amp=0.22)),
        ("idle", dict(beats_per_second=0.5, arm_amp=0.15, sway_amp=0.03)),
    ]
    return [(name, retarget(generate_dance(duration=2.0, **kw), morph)) for name, kw in specs]


def _demo_track_multi(out: Path, robot: str, iterations: int, stride: int) -> int:
    """1 つの方策で参照スイート（4 運動）を物理追従し、横並び描画する（§4.5 汎化）。"""
    from robotdance_models.tracking_policy import train_multi_tracking_policy
    from robotdance_unitree import get_morphology
    from robotdance_viewer.skeleton_view import render_side_by_side

    morph = get_morphology(robot)
    suite = _tracking_suite(morph)
    refs = [r for _, r in suite]
    print(f"🤹 1 つの RL 方策で {len(suite)} 運動スイートを追従学習中"
          f"（{robot}, PPO {iterations} iters, round-robin）...")
    policy, info = train_multi_tracking_policy(refs, morph, iterations=iterations)
    print(f"  episode return: {info['return_history'][0]:.2f} → {info['return_history'][-1]:.2f}")

    colors = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd"]
    panels, verdicts = [], []
    for i, (name, _ref) in enumerate(suite):
        motion, m = policy.rollout(i)
        print(f"  {name:7s}: 生存 {m['survived_frames']}/{m['reference_frames']} "
              f"(survival {m['survival_ratio']:.0%}) / pose RMSE {m['mean_pose_rmse']:.3f}")
        panels.append((motion.keypoints_3d_array(), name, colors[i % len(colors)]))
        verdicts.append((f"{name} {m['survival_ratio']:.0%}", colors[i % len(colors)]))
    render_side_by_side(panels, out, stride=stride, verdicts=verdicts)
    print(f"✓ multi-motion tracking デモ GIF: {out}")
    print("  ⚠️ v0 baseline: 1 方策が運動に応じ追従（reference-conditioned）。"
          "短い feasible クリップでは PD でも概ねバランスするため PD 超えは主張しない。")
    return 0


def _search_text(query: str, checkpoint: Path, k: int) -> int:
    """テキスト query から合成モーション・スイートを意味検索する（§4.2 デモ）。"""
    from robotdance_core.synthetic import generate_backflip, generate_dance
    from robotdance_models.contrastive import TextMotionModel

    motions = {
        "dance_fast": generate_dance(beats_per_second=1.6),
        "dance_normal": generate_dance(beats_per_second=1.0),
        "dance_slow": generate_dance(beats_per_second=0.7),
        "idle": generate_dance(beats_per_second=0.5, arm_amp=0.15, sway_amp=0.04),
        "backflip": generate_backflip(duration=1.6),
    }
    model = TextMotionModel(checkpoint)
    print(f'🔎 query: "{query}"')
    for mid, sim in model.search(query, motions, k=k):
        bar = "█" * round(max(sim, 0.0) * 20)
        print(f"  {mid:14s} cos={sim:+.3f} {bar}")
    return 0


def _demo_motion_map(out: Path, checkpoint: Path | None = None) -> int:
    """多様な合成モーションを埋め込み、検索・重複・2D マップを示す（§6.2 Demo 3）。

    checkpoint を渡すと学習 encoder（robotdance_models）、無ければ手作り embedding を使う。
    """
    from .synthetic import generate_backflip, generate_dance
    from robotdance_motion.embeddings import MotionIndex, embed
    from robotdance_viewer.motion_map import render_motion_map

    embed_fn = embed
    tag = "hand-crafted"
    if checkpoint is not None:
        from robotdance_models.train import LearnedMotionEncoder

        embed_fn = LearnedMotionEncoder(checkpoint).embed
        tag = f"learned({checkpoint.name})"

    specs = {
        "dance_normal": generate_dance(beats_per_second=1.0),
        "dance_fast": generate_dance(beats_per_second=1.6),
        "dance_slow": generate_dance(beats_per_second=0.7),
        "dance_dup": generate_dance(beats_per_second=1.0),  # dance_normal とほぼ同一
        "idle_a": generate_dance(beats_per_second=0.5, arm_amp=0.15, sway_amp=0.04),
        "idle_b": generate_dance(beats_per_second=0.6, arm_amp=0.20, sway_amp=0.05),
        "backflip_a": generate_backflip(duration=1.6),
        "backflip_b": generate_backflip(duration=1.4),
    }
    index = MotionIndex(embed_fn=embed_fn)
    for mid, mir in specs.items():
        mir.motion_id = mid
        index.add_mir(mir)

    labels = list(index.ids)
    groups = [lab.split("_")[0] for lab in labels]  # dance / idle / backflip
    render_motion_map(index.project_2d(), labels, out, groups=groups,
                      title=f"RobotDance Motion Map ({tag})")

    print(f"✓ Motion Map [{tag}]: {out}")
    print("  retrieval（query=dance_fast）:")
    for mid, sim in index.query(embed_fn(specs["dance_fast"]), k=3):
        print(f"    {mid:14s} cos={sim:.3f}")
    print("  near-duplicates (>=0.98):")
    for a, b, s in index.duplicates(0.98):
        print(f"    {a} ~ {b}  cos={s:.4f}")
    return 0


def _build_dataset(manifest_file: Path, data_root: Path, out_dir: Path, dedupe: bool) -> int:
    from robotdance_data.dataset import build_from_file

    report = build_from_file(manifest_file, data_root=data_root, out_dir=out_dir, dedupe=dedupe)
    print(f"✓ dataset build: exported {report['exported']} / withheld {report['withheld']} "
          f"(total {report['total']})")
    print(f"  Data Bill of Materials: {out_dir / 'DATA_CARD.md'}")
    for r in report["bill_of_materials"]:
        mark = "✅" if r["exported"] else "⛔"
        print(f"  {mark} {r['clip_id']:16s} [{r['license_state']}] {r['reason']}")
    return 0


def _smooth(path: Path, out: Path, window: int) -> int:
    from .rd_mir import RdMir
    from robotdance_motion.smoothing import smooth_rdmir

    mir = smooth_rdmir(RdMir.load(path), window=window)
    mir.save(out)
    qm = mir.quality_metrics or {}
    print(f"✓ 平滑化 RD-MIR: {out}")
    print(f"  jitter {qm.get('jitter_before')} → {qm.get('jitter_after')} ({qm.get('smoothing')})")
    return 0


def _demo_smoothing(out: Path, stride: int) -> int:
    """jittery な抽出を想定し、raw(noisy) vs smoothed を side-by-side で示す。"""
    from .synthetic import generate_dance
    from robotdance_motion.smoothing import add_jitter, jitter, smooth_rdmir
    from robotdance_viewer.skeleton_view import render_side_by_side

    noisy = add_jitter(generate_dance(duration=4.0), sigma=0.025)
    smoothed = smooth_rdmir(noisy)
    j_raw = jitter(noisy.keypoints_3d_array())
    j_smooth = jitter(smoothed.keypoints_3d_array())
    render_side_by_side(
        [
            (noisy.keypoints_3d_array(), "raw (jittery)", "#d62728"),
            (smoothed.keypoints_3d_array(), "smoothed (Savitzky-Golay)", "#2ca02c"),
        ],
        out, fps=noisy.fps, stride=stride,
        verdicts=[(f"jitter {j_raw:.3f}", "#d62728"), (f"jitter {j_smooth:.3f}", "#2ca02c")],
    )
    print(f"✓ smoothing デモ GIF: {out}  jitter {j_raw:.4f} → {j_smooth:.4f}")
    return 0


def _overlay(video: Path, mir_path: Path, out: Path, stride: int) -> int:
    from .rd_mir import RdMir
    from robotdance_viewer.overlay import render_overlay

    render_overlay(video, RdMir.load(mir_path), out, stride=stride)
    print(f"✓ overlay GIF（原動画 + 骨格）: {out}")
    return 0


def _validate_sim(path: Path, robot: str, out: Path | None) -> int:
    from .rd_mir import RdMir
    from robotdance_retarget.kinematic import retarget
    from robotdance_sim.mujoco_backend import certify
    from robotdance_unitree import get_morphology

    morph = get_morphology(robot)
    motion = retarget(RdMir.load(path), morph)
    certify(motion, morph)
    cert = motion.sim_certificate or {}
    print(f"{'✅' if cert.get('passed') else '⛔'} {robot}: {cert.get('verdict')}")
    for k, v in (cert.get("metrics") or {}).items():
        print(f"    {k} = {v}")
    for r in cert.get("reasons") or []:
        print(f"    ⚠️ {r}")
    print(f"    {cert.get('note')}")
    if out is not None:
        motion.save(out)
        print(f"  → sim_certificate 付き RD-Motion を保存: {out}")
    return 0 if cert.get("passed") else 1


def _demo_safety(out: Path, robot: str, stride: int) -> int:
    """安全なダンス(PASS) と バックフリップ(REJECT) を side-by-side で示す（§6.2 Demo 4）。"""
    from .synthetic import generate_backflip, generate_dance
    from robotdance_retarget.kinematic import retarget
    from robotdance_sim.mujoco_backend import certify
    from robotdance_unitree import get_morphology
    from robotdance_viewer.skeleton_view import render_side_by_side

    morph = get_morphology(robot)
    panels, verdicts = [], []
    for label, mir in [("dance", generate_dance(duration=4.0)), ("backflip", generate_backflip())]:
        motion = retarget(mir, morph)
        certify(motion, morph)
        cert = motion.sim_certificate
        verdict = cert["verdict"]
        color = "#2ca02c" if cert["passed"] else "#d62728"
        panels.append((motion.keypoints_3d_array(), f"{label} → {robot}", "#ff7f0e"))
        verdicts.append((verdict, color))
        print(f"  {label:9s} → {verdict}  reasons={cert['reasons']}")
    render_side_by_side(panels, out, fps=30.0, stride=stride, verdicts=verdicts)
    print(f"✓ safety デモ GIF を書き出しました: {out}")
    return 0


# multi-embodiment デモで描き分ける色。
_ROBOT_COLORS = {"unitree_g1": "#ff7f0e", "unitree_h1": "#2ca02c"}


def _demo_multi(out: Path, robots: list[str], stride: int) -> int:
    """synth → 複数ロボットへ retarget → "Same motion, many humanoids" を一括描画。"""
    from .synthetic import generate_dance
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology
    from robotdance_viewer.skeleton_view import render_side_by_side

    mir = generate_dance()
    panels = [(mir.keypoints_3d_array(), "Human (RD-MIR)", "#1f77b4")]
    for name in robots:
        motion = retarget(mir, get_morphology(name))
        m = motion.retarget_metrics or {}
        panels.append((motion.keypoints_3d_array(), name, _ROBOT_COLORS.get(name, "#9467bd")))
        print(f"  {name}: height_scale={m.get('height_scale')} "
              f"foot_sliding={m.get('foot_sliding_m_per_frame')}")
    render_side_by_side(panels, out, fps=mir.fps, stride=stride)
    print(f"✓ multi-embodiment デモ GIF を書き出しました: {out}（{len(panels)} panels）")
    print("  ⚠️ kinematic preview のみ — 物理 sim 未検証（Phase 2）")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="robotdance", description="RobotDance core CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    p_validate = sub.add_parser("validate", help="JSON を RobotDance spec で検証する")
    p_validate.add_argument("spec", choices=sorted(_SCHEMAS), help="検証する spec")
    p_validate.add_argument("path", type=Path, help="検証対象 JSON ファイル")

    p_synth = sub.add_parser("synth", help="合成ダンスモーション RD-MIR を生成する")
    p_synth.add_argument("-o", "--out", type=Path, default=Path("synthetic_dance.rdmir.json"))
    p_synth.add_argument("--duration", type=float, default=4.0)
    p_synth.add_argument("--fps", type=float, default=30.0)

    p_view = sub.add_parser("view", help="RD-MIR を 3D スケルトン GIF に描画する")
    p_view.add_argument("path", type=Path, help="RD-MIR JSON")
    p_view.add_argument("-o", "--out", type=Path, default=Path("skeleton.gif"))
    p_view.add_argument("--stride", type=int, default=2, help="何フレームおきに描画するか")

    p_ret = sub.add_parser("retarget", help="RD-MIR を Unitree ロボットへ kinematic retarget する")
    p_ret.add_argument("path", type=Path, help="RD-MIR JSON")
    p_ret.add_argument("-o", "--out", type=Path, default=Path("robot.rdmotion.json"))
    p_ret.add_argument("--robot", default="unitree_g1",
                       help="対象ロボット（unitree_g1 / unitree_h1）")

    p_pair = sub.add_parser("view-pair", help="human RD-MIR と robot RD-Motion を side-by-side 描画")
    p_pair.add_argument("human", type=Path, help="RD-MIR JSON")
    p_pair.add_argument("robot", type=Path, help="RD-Motion JSON")
    p_pair.add_argument("-o", "--out", type=Path, default=Path("side_by_side.gif"))
    p_pair.add_argument("--stride", type=int, default=2)

    p_demo = sub.add_parser("demo-g1", help="synth → retarget → side-by-side を一括実行")
    p_demo.add_argument("-o", "--out", type=Path, default=Path("g1_side_by_side.gif"))
    p_demo.add_argument("--stride", type=int, default=2)

    p_multi = sub.add_parser("demo-multi", help="synth → 複数ロボット retarget → 横並び描画")
    p_multi.add_argument("-o", "--out", type=Path, default=Path("many_humanoids.gif"))
    p_multi.add_argument("--robots", nargs="+", default=["unitree_g1", "unitree_h1"])
    p_multi.add_argument("--stride", type=int, default=2)

    p_serve = sub.add_parser("serve", help=".rdmotion を safety guard 越しに再生（--ros2 で ROS2 配信）")
    p_serve.add_argument("rdmotion", type=Path, help="certified .rdmotion JSON")
    p_serve.add_argument("--speed", type=float, default=1.0)
    p_serve.add_argument("--ros2", action="store_true", help="ROS2 ノードとして配信（rclpy 必要）")
    p_serve.add_argument("--allow-uncertified", action="store_true", help="certificate 無しでも再生（危険）")

    sub.add_parser("demo-runtime", help="safety guard の PASS/REJECT 遮断を実演")

    p_bench = sub.add_parser("benchmark", help="motion × robot を回し CSV + leaderboard を出力")
    p_bench.add_argument("--robots", nargs="+", default=["unitree_g1", "unitree_h1"])
    p_bench.add_argument("--motions-dir", type=Path, default=None, help="*.rdmir.json のディレクトリ（既定: 合成スイート）")
    p_bench.add_argument("--no-sim", action="store_true", help="MuJoCo 物理検証を行わない")
    p_bench.add_argument("-o", "--out", type=Path, default=Path("benchmark_out"))

    p_mmap = sub.add_parser("demo-motion-map", help="合成モーションを埋め込み Motion Map を描く")
    p_mmap.add_argument("-o", "--out", type=Path, default=Path("motion_map.png"))
    p_mmap.add_argument("--checkpoint", type=Path, default=None,
                        help="学習 encoder の .pt（省略時は手作り embedding）")

    p_ik = sub.add_parser("retarget-ik", help="RD-MIR を実 URDF のアクチュエータ関節角へ IK retarget")
    p_ik.add_argument("path", type=Path, help="RD-MIR JSON")
    p_ik.add_argument("--urdf", type=Path, required=True, help="ロボット URDF（例: g1_23dof.urdf）")
    p_ik.add_argument("-o", "--out", type=Path, default=Path("g1_joints.rdmotion.json"))
    p_ik.add_argument("--steps", type=int, default=300)

    p_urdf = sub.add_parser("import-urdf", help="実 URDF から実寸 RobotMorphology を構築する")
    p_urdf.add_argument("urdf", type=Path, help="URDF ファイル（例: g1_23dof.urdf）")
    p_urdf.add_argument("--name", default="unitree_g1")
    p_urdf.add_argument("--save", type=Path, default=None, help="RD-Embodiment JSON 保存先")

    p_train = sub.add_parser("train-encoder", help="masked motion modeling encoder を学習する")
    p_train.add_argument("-o", "--out", type=Path, default=Path("motion_encoder.pt"))
    p_train.add_argument("--epochs", type=int, default=40)
    p_train.add_argument("--device", default=None, help="cpu / cuda（既定: 自動）")

    p_tm = sub.add_parser("train-text-motion", help="contrastive text-motion encoder を学習する")
    p_tm.add_argument("-o", "--out", type=Path, default=Path("text_motion.pt"))
    p_tm.add_argument("--epochs", type=int, default=120)
    p_tm.add_argument("--device", default=None, help="cpu / cuda（既定: 自動）")

    p_st = sub.add_parser("search-text", help="テキストで合成モーションを意味検索する")
    p_st.add_argument("query", help='検索文（例: "a person doing a backflip"）')
    p_st.add_argument("--checkpoint", type=Path, default=Path("text_motion.pt"),
                      help="train-text-motion の .pt")
    p_st.add_argument("-k", type=int, default=5)

    p_tok = sub.add_parser("train-tokenizer", help="motion VQ-VAE（離散トークナイザ）を学習する")
    p_tok.add_argument("-o", "--out", type=Path, default=Path("motion_tokenizer.pt"))
    p_tok.add_argument("--epochs", type=int, default=150)
    p_tok.add_argument("--codes", type=int, default=128, help="codebook サイズ")
    p_tok.add_argument("--device", default=None, help="cpu / cuda（既定: 自動）")

    p_dtok = sub.add_parser("demo-tokenizer", help="motion をトークン化し圧縮・再構成を実演")
    p_dtok.add_argument("-o", "--out", type=Path, default=Path("tokenizer_recon.gif"))
    p_dtok.add_argument("--checkpoint", type=Path, default=None,
                        help="train-tokenizer の .pt（省略時はその場で学習）")
    p_dtok.add_argument("--epochs", type=int, default=150)
    p_dtok.add_argument("--stride", type=int, default=2)

    p_pri = sub.add_parser("train-prior", help="VQ-VAE トークン列の生成 prior を学習する")
    p_pri.add_argument("--tokenizer", type=Path, default=Path("motion_tokenizer.pt"),
                       help="train-tokenizer の .pt")
    p_pri.add_argument("-o", "--out", type=Path, default=Path("motion_prior.pt"))
    p_pri.add_argument("--epochs", type=int, default=300)
    p_pri.add_argument("--device", default=None, help="cpu / cuda（既定: 自動）")

    p_dgen = sub.add_parser("demo-generate", help="token prior でモーションを生成・補完")
    p_dgen.add_argument("-o", "--out", type=Path, default=Path("generated.gif"))
    p_dgen.add_argument("--checkpoint", type=Path, default=None,
                        help="train-prior の .pt（省略時はその場で学習）")
    p_dgen.add_argument("--epochs", type=int, default=150)
    p_dgen.add_argument("--temperature", type=float, default=1.0)
    p_dgen.add_argument("--stride", type=int, default=2)

    p_t2m = sub.add_parser("train-text2motion", help="テキスト条件付き生成 prior を学習する")
    p_t2m.add_argument("--tokenizer", type=Path, default=Path("motion_tokenizer.pt"),
                       help="train-tokenizer の .pt")
    p_t2m.add_argument("-o", "--out", type=Path, default=Path("text2motion.pt"))
    p_t2m.add_argument("--epochs", type=int, default=400)
    p_t2m.add_argument("--device", default=None, help="cpu / cuda（既定: 自動）")

    p_gt = sub.add_parser("generate-text", help="テキストからモーションを生成する")
    p_gt.add_argument("caption", help='生成したい動作の説明（例: "a person doing a backflip"）')
    p_gt.add_argument("--checkpoint", type=Path, default=Path("text2motion.pt"),
                      help="train-text2motion の .pt")
    p_gt.add_argument("-o", "--out", type=Path, default=Path("generated.rdmir.json"))
    p_gt.add_argument("--gif", type=Path, default=None, help="生成モーションの GIF 出力先")
    p_gt.add_argument("--temperature", type=float, default=0.8)
    p_gt.add_argument("--seed", type=int, default=0)
    p_gt.add_argument("--stride", type=int, default=2)

    p_build = sub.add_parser("build-dataset", help="RD-Manifest から RD-MIR を構築（license firewall）")
    p_build.add_argument("manifest", type=Path, help="manifest JSON（配列 or 単体）")
    p_build.add_argument("--data-root", type=Path, default=Path("."), help="ローカル source の基準ディレクトリ")
    p_build.add_argument("--dedupe", action="store_true", help="motion embedding で near-duplicate を除去")
    p_build.add_argument("-o", "--out", type=Path, default=Path("build"))

    p_smooth = sub.add_parser("smooth", help="RD-MIR を Savitzky-Golay で平滑化する")
    p_smooth.add_argument("path", type=Path, help="RD-MIR JSON")
    p_smooth.add_argument("-o", "--out", type=Path, default=Path("smoothed.rdmir.json"))
    p_smooth.add_argument("--window", type=int, default=7)

    p_dsmooth = sub.add_parser("demo-smoothing", help="raw(noisy) vs smoothed を side-by-side")
    p_dsmooth.add_argument("-o", "--out", type=Path, default=Path("smoothing.gif"))
    p_dsmooth.add_argument("--stride", type=int, default=2)

    p_overlay = sub.add_parser("overlay", help="原動画に RD-MIR の 2D 骨格を重ねて GIF 化")
    p_overlay.add_argument("video", type=Path, help="原動画")
    p_overlay.add_argument("mir", type=Path, help="extract で得た RD-MIR JSON")
    p_overlay.add_argument("-o", "--out", type=Path, default=Path("overlay.gif"))
    p_overlay.add_argument("--stride", type=int, default=2)

    p_extract = sub.add_parser("extract", help="local 動画から MediaPipe で RD-MIR を抽出")
    p_extract.add_argument("video", type=Path, help="入力動画（ローカルファイル）")
    p_extract.add_argument("-o", "--out", type=Path, default=Path("video.rdmir.json"))
    p_extract.add_argument("--model", type=Path, default=None, help="pose model (.task) パス")

    p_hmr = sub.add_parser("import-hmr",
                           help="HMR(4DHumans/GVHMR)の SMPL 出力(.npz)を RD-MIR 化（§4.1）")
    p_hmr.add_argument("path", type=Path, help="HMR 出力 .npz（global_orient/body_pose[/transl/fps]）")
    p_hmr.add_argument("--source", default="hmr", help="4dhumans / gvhmr / hmr（メタ表示用）")
    p_hmr.add_argument("--fps", type=float, default=None, help="フレームレート（npz に無ければ指定）")
    p_hmr.add_argument("-o", "--out", type=Path, default=Path("hmr.rdmir.json"))

    p_v2r = sub.add_parser("video-to-robot", help="動画 → RD-MIR → retarget → 物理検証 → side-by-side")
    p_v2r.add_argument("video", type=Path, help="入力動画（ローカルファイル）")
    p_v2r.add_argument("--robot", default="unitree_g1")
    p_v2r.add_argument("-o", "--out", type=Path, default=Path("shorts_to_humanoid.gif"))
    p_v2r.add_argument("--stride", type=int, default=2)

    p_vsim = sub.add_parser("validate-sim", help="RD-MIR を robot へ retarget し MuJoCo 物理検証")
    p_vsim.add_argument("path", type=Path, help="RD-MIR JSON")
    p_vsim.add_argument("--robot", default="unitree_g1")
    p_vsim.add_argument("-o", "--out", type=Path, default=None, help="certificate 付き .rdmotion 保存先")

    p_safety = sub.add_parser("demo-safety", help="safe dance(PASS) vs backflip(REJECT) を描画")
    p_safety.add_argument("-o", "--out", type=Path, default=Path("safety_check.gif"))
    p_safety.add_argument("--robot", default="unitree_g1")
    p_safety.add_argument("--stride", type=int, default=2)

    p_trk = sub.add_parser("train-tracking", help="RL tracking policy（物理上で参照を追従）を学習")
    p_trk.add_argument("-o", "--out", type=Path, default=Path("tracking_policy.pt"))
    p_trk.add_argument("--robot", default="unitree_g1")
    p_trk.add_argument("--iterations", type=int, default=40)
    p_trk.add_argument("--suite", action="store_true", help="複数運動を 1 方策に汎化（multi-motion）")
    p_trk.add_argument("--device", default=None, help="cpu / cuda（既定: 自動）")

    p_dtrk = sub.add_parser("demo-track", help="参照を RL policy で物理追従し side-by-side 描画")
    p_dtrk.add_argument("-o", "--out", type=Path, default=Path("tracking.gif"))
    p_dtrk.add_argument("--robot", default="unitree_g1")
    p_dtrk.add_argument("--iterations", type=int, default=40)
    p_dtrk.add_argument("--stride", type=int, default=2)

    p_dtrkm = sub.add_parser("demo-track-multi",
                             help="1 方策で運動スイート（4 運動）を物理追従し横並び描画")
    p_dtrkm.add_argument("-o", "--out", type=Path, default=Path("tracking_multi.gif"))
    p_dtrkm.add_argument("--robot", default="unitree_g1")
    p_dtrkm.add_argument("--iterations", type=int, default=60)
    p_dtrkm.add_argument("--stride", type=int, default=2)

    sub.add_parser("demo-joint-safety",
                   help="関節空間 safety guard の位置/速度/加速度クランプを実演（§5.6）")

    args = parser.parse_args(argv)
    if args.command == "validate":
        return _validate(args.spec, args.path)
    if args.command == "synth":
        return _synth(args.out, args.duration, args.fps)
    if args.command == "view":
        return _view(args.path, args.out, args.stride)
    if args.command == "retarget":
        return _retarget(args.path, args.out, args.robot)
    if args.command == "view-pair":
        return _view_pair(args.human, args.robot, args.out, args.stride)
    if args.command == "demo-g1":
        return _demo_g1(args.out, args.stride)
    if args.command == "demo-multi":
        return _demo_multi(args.out, args.robots, args.stride)
    if args.command == "validate-sim":
        return _validate_sim(args.path, args.robot, args.out)
    if args.command == "demo-safety":
        return _demo_safety(args.out, args.robot, args.stride)
    if args.command == "train-tracking":
        return _train_tracking(args.out, args.robot, args.iterations, args.device, args.suite)
    if args.command == "demo-track":
        return _demo_track(args.out, args.robot, args.iterations, args.stride)
    if args.command == "demo-track-multi":
        return _demo_track_multi(args.out, args.robot, args.iterations, args.stride)
    if args.command == "demo-joint-safety":
        return _demo_joint_safety()
    if args.command == "serve":
        return _serve(args.rdmotion, args.speed, args.ros2, args.allow_uncertified)
    if args.command == "demo-runtime":
        return _demo_runtime()
    if args.command == "benchmark":
        return _benchmark(args.robots, args.motions_dir, not args.no_sim, args.out)
    if args.command == "demo-motion-map":
        return _demo_motion_map(args.out, args.checkpoint)
    if args.command == "retarget-ik":
        return _retarget_ik(args.path, args.urdf, args.out, args.steps)
    if args.command == "import-urdf":
        return _import_urdf(args.urdf, args.name, args.save)
    if args.command == "train-encoder":
        return _train_encoder(args.out, args.epochs, args.device)
    if args.command == "train-text-motion":
        return _train_text_motion(args.out, args.epochs, args.device)
    if args.command == "search-text":
        return _search_text(args.query, args.checkpoint, args.k)
    if args.command == "train-tokenizer":
        return _train_tokenizer(args.out, args.epochs, args.codes, args.device)
    if args.command == "demo-tokenizer":
        return _demo_tokenizer(args.out, args.checkpoint, args.epochs, args.stride)
    if args.command == "train-prior":
        return _train_prior(args.tokenizer, args.out, args.epochs, args.device)
    if args.command == "demo-generate":
        return _demo_generate(args.out, args.checkpoint, args.epochs, args.temperature, args.stride)
    if args.command == "train-text2motion":
        return _train_text2motion(args.tokenizer, args.out, args.epochs, args.device)
    if args.command == "generate-text":
        return _generate_text(args.caption, args.checkpoint, args.out, args.gif,
                              args.temperature, args.seed, args.stride)
    if args.command == "build-dataset":
        return _build_dataset(args.manifest, args.data_root, args.out, args.dedupe)
    if args.command == "smooth":
        return _smooth(args.path, args.out, args.window)
    if args.command == "demo-smoothing":
        return _demo_smoothing(args.out, args.stride)
    if args.command == "overlay":
        return _overlay(args.video, args.mir, args.out, args.stride)
    if args.command == "extract":
        return _extract(args.video, args.out, args.model)
    if args.command == "import-hmr":
        return _import_hmr(args.path, args.source, args.fps, args.out)
    if args.command == "video-to-robot":
        return _video_to_robot(args.video, args.robot, args.out, args.stride)
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
