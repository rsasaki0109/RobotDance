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
    if args.command == "video-to-robot":
        return _video_to_robot(args.video, args.robot, args.out, args.stride)
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
