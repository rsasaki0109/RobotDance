"""retarget / sim benchmark ハーネス（v0）。

モーション集合 × ロボット集合を full pipeline（retarget → MuJoCo 物理検証）に通し、
既存の全指標（retarget metrics / sim_certificate / source quality）を 1 行 = 1 (motion, robot)
に集約する。出力は CSV + Markdown leaderboard（robotdance_benchmarks.report）。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from robotdance_core.rd_mir import RdMir
from robotdance_retarget.kinematic import retarget
from robotdance_unitree import get_morphology


@dataclass
class BenchmarkRow:
    """1 (motion, robot) の benchmark 結果。"""

    motion_id: str
    motion_class: str
    robot: str
    # retarget（常に算出）
    height_scale: Optional[float] = None
    bone_direction_cosine: Optional[float] = None
    foot_sliding: Optional[float] = None
    # 膝・肘の屈曲が実 per-joint 可動域上限を超えたフレーム比（per_joint_limits 有時のみ, v0.39+）
    joint_flexion_violation: Optional[float] = None
    # sim_certificate（mujoco があれば）
    verdict: Optional[str] = None
    airborne_ratio: Optional[float] = None
    balance_violation_ratio: Optional[float] = None
    torque_ratio: Optional[float] = None
    max_joint_ang_speed: Optional[float] = None
    # source 品質（RD-MIR の quality_metrics から）
    source_confidence: Optional[float] = None
    jitter: Optional[float] = None


def _motion_class(mir: RdMir) -> str:
    sem = mir.semantics or {}
    if sem.get("action_label") and sem["action_label"] != "unknown":
        return str(sem["action_label"])
    return mir.motion_id.split("_")[0].split("-")[-1]


def default_motion_suite() -> dict[str, RdMir]:
    """合成モーションの標準スイート（権利クリーン・決定的）。"""
    from robotdance_core.synthetic import generate_backflip, generate_dance

    suite = {
        "dance_normal": generate_dance(beats_per_second=1.0),
        "dance_fast": generate_dance(beats_per_second=1.6),
        "idle": generate_dance(beats_per_second=0.5, arm_amp=0.15, sway_amp=0.04),
        "backflip": generate_backflip(duration=1.6),
    }
    for name, mir in suite.items():
        mir.motion_id = name
    return suite


def run_benchmark(
    motions: dict[str, RdMir],
    robots: list[str],
    *,
    with_sim: bool = True,
) -> dict:
    """motions × robots を回し、行リストと集計を返す。

    with_sim=True でも mujoco 未インストールなら sim 指標は None になる（degrade gracefully）。
    """
    sim_available = with_sim
    certify = None
    if with_sim:
        try:
            from robotdance_sim.mujoco_backend import certify as _certify

            certify = _certify
        except ImportError:
            sim_available = False

    rows: list[BenchmarkRow] = []
    for name, mir in motions.items():
        qm = mir.quality_metrics or {}
        for robot in robots:
            morph = get_morphology(robot)
            motion = retarget(mir, morph)
            rm = motion.retarget_metrics or {}
            row = BenchmarkRow(
                motion_id=name,
                motion_class=_motion_class(mir),
                robot=robot,
                height_scale=rm.get("height_scale"),
                bone_direction_cosine=rm.get("bone_direction_cosine"),
                foot_sliding=rm.get("foot_sliding_m_per_frame"),
                joint_flexion_violation=(rm.get("joint_flexion") or {}).get("any_violation_ratio"),
                source_confidence=qm.get("mean_confidence"),
                jitter=qm.get("jitter_after", qm.get("jitter_before")),
            )
            if sim_available and certify is not None:
                certify(motion, morph)
                cert = motion.sim_certificate or {}
                cm = cert.get("metrics", {})
                row.verdict = cert.get("verdict")
                row.airborne_ratio = cm.get("airborne_ratio")
                row.balance_violation_ratio = cm.get("balance_violation_ratio")
                row.torque_ratio = cm.get("torque_ratio")
                row.max_joint_ang_speed = cm.get("max_joint_ang_speed_rad_s")
            rows.append(row)

    return {
        "rows": [asdict(r) for r in rows],
        "robots": list(robots),
        "motions": list(motions),
        "sim_available": sim_available,
    }


def run_from_dir(
    motions_dir: str | Path, robots: list[str], *, with_sim: bool = True
) -> dict:
    """ディレクトリ内の *.rdmir.json を読み込んで benchmark する。"""
    motions: dict[str, RdMir] = {}
    for p in sorted(Path(motions_dir).glob("*.rdmir.json")):
        mir = RdMir.load(p)
        motions[p.stem.replace(".rdmir", "")] = mir
    return run_benchmark(motions, robots, with_sim=with_sim)
