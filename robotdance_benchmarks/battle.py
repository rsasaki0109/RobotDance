"""HumanoidBattle — 2 体のヒューマノイドが同じ/別のモーションを実行し、**実行品質**で 1v1 採点する。

格闘ゲーム的な「殴り合い」（接触ダイナミクス）は v0 では物理化しない。代わりに**型（kata）/演武
バトル**として、各ファイターが motion を実機へ retarget した結果を、RobotDance が既に算出する
**実 metrics**（reach 誤差・bone 方向・foot sliding・関節可動域、任意で sim の balance/torque）から
0〜100 点の透明なスコアに合成し、高い方を勝者とする。体格差（G1 1.29m vs H1 1.66m）が実行品質に
出る——同じ kata でも「どの体がきれいに再現できるか」の勝負。スコアは乱数でなく**全て実測由来**。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from robotdance_core.synthetic import (
    generate_backflip,
    generate_dance,
    generate_march,
    generate_overbend,
    generate_squat,
)

# モーション名 → 合成ジェネレータ。CLI から `robot:motion` で指定する。
MOTIONS = {
    "dance": generate_dance, "kata": generate_dance, "march": generate_march,
    "squat": generate_squat, "backflip": generate_backflip, "bow": generate_overbend,
}


def _clip01(x: float) -> float:
    return float(min(1.0, max(0.0, x)))


@dataclass
class Scorecard:
    """1 ファイターの採点内訳（各 0〜100, overall は加重平均）。"""

    overall: float
    breakdown: dict[str, float] = field(default_factory=dict)


def score_fighter(retarget_metrics: dict, sim_cert: dict | None = None) -> Scorecard:
    """retarget（+任意 sim）metrics から実行品質スコアを合成する。全項目 0〜100、高いほど良い。

    透明性のため各コンポーネントを breakdown に残す。重みは下記コメントの通り（合計 1.0）。
    """
    rm = retarget_metrics or {}
    comp: dict[str, float] = {}

    # fidelity（型の正確さ）。
    reach = rm.get("endeffector_reach_error_m")
    if reach is not None:
        comp["reach"] = 100.0 * _clip01(1.0 - reach / 0.25)        # 0m→100, 0.25m→0
    comp["form"] = 100.0 * _clip01(rm.get("bone_direction_cosine", 1.0))  # bone 方向一致

    # stability（接地の安定）。
    slide = rm.get("foot_sliding_m_per_frame")
    if slide is not None:
        comp["footwork"] = 100.0 * _clip01(1.0 - slide / 0.02)     # 滑らないほど高い

    # feasibility（実機可動域）。
    jf = rm.get("joint_flexion") or {}
    viol = jf.get("any_violation_ratio")
    if viol is not None:
        comp["control"] = 100.0 * _clip01(1.0 - viol)

    # 任意: 物理 sim（balance / torque）。
    if sim_cert:
        m = sim_cert.get("metrics") or {}
        if "balance_violation_ratio" in m:
            comp["balance"] = 100.0 * _clip01(1.0 - m["balance_violation_ratio"])
        if "torque_ratio" in m:
            comp["power"] = 100.0 * _clip01(1.0 - max(0.0, m["torque_ratio"] - 1.0))
        if not sim_cert.get("passed", True):
            comp["balance"] = comp.get("balance", 50.0) * 0.5      # REJECT は減点

    # 重み（存在する項目だけで正規化）。fidelity 0.4 / stability 0.3 / feasibility 0.3。
    weights = {"reach": 0.25, "form": 0.15, "footwork": 0.30,
               "control": 0.15, "balance": 0.10, "power": 0.05}
    num = sum(comp[k] * weights[k] for k in comp)
    den = sum(weights[k] for k in comp) or 1.0
    return Scorecard(overall=round(num / den, 1), breakdown={k: round(v, 1) for k, v in comp.items()})


@dataclass
class BattleResult:
    p1_name: str
    p2_name: str
    p1_card: Scorecard
    p2_card: Scorecard
    winner: str          # p1_name / p2_name / "DRAW"
    p1_kps: np.ndarray = field(repr=False, default=None)
    p2_kps: np.ndarray = field(repr=False, default=None)
    fps: float = 30.0


def _parse_fighter(spec: str) -> tuple[str, str]:
    """`robot:motion` をパース。motion 省略時は kata（= dance）。"""
    if ":" in spec:
        robot, motion = spec.split(":", 1)
    else:
        robot, motion = spec, "kata"
    if motion not in MOTIONS:
        raise ValueError(f"未知の motion '{motion}'（利用可能: {sorted(MOTIONS)}）")
    return robot, motion


def run_battle(p1_spec: str, p2_spec: str, *, sim: bool = False) -> BattleResult:
    """2 ファイター（`robot:motion`）を retarget→採点し、勝者を決める。"""
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    cards = []
    kps = []
    names = []
    fps = 30.0
    for spec in (p1_spec, p2_spec):
        robot, motion = _parse_fighter(spec)
        mir = MOTIONS[motion]()
        fps = float(mir.fps)
        mo = retarget(mir, get_morphology(robot))
        cert = None
        if sim:
            from robotdance_sim.backend import certify
            certify(mo, get_morphology(robot))
            cert = mo.sim_certificate
        cards.append(score_fighter(mo.retarget_metrics or {}, cert))
        kps.append(mo.keypoints_3d_array())
        names.append(f"{robot}:{motion}")

    if cards[0].overall > cards[1].overall:
        winner = names[0]
    elif cards[1].overall > cards[0].overall:
        winner = names[1]
    else:
        winner = "DRAW"
    return BattleResult(names[0], names[1], cards[0], cards[1], winner,
                        kps[0], kps[1], fps=fps)


__all__ = ["Scorecard", "BattleResult", "score_fighter", "run_battle", "MOTIONS"]
