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
