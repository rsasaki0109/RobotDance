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

from robotdance_benchmarks.real_motions import REAL_MOTIONS
from robotdance_sim.fight_moves import generate_dodge, generate_hook, generate_kick
from robotdance_core.synthetic import (
    generate_backflip,
    generate_dance,
    generate_march,
    generate_overbend,
    generate_squat,
)

# モーション名 → 合成ジェネレータ or 実動画フィクスチャ loader。CLI から `robot:motion` で指定。
MOTIONS = {
    "dance": generate_dance, "kata": generate_dance, "march": generate_march,
    "squat": generate_squat, "backflip": generate_backflip, "bow": generate_overbend,
    "hook": generate_hook, "kick": generate_kick, "dodge": generate_dodge,
    **REAL_MOTIONS,
}

# 技の難度＝style 倍率（透明な「リスク/リターン」）。難しい技ほど決まれば高得点だが、自分の体で
# 物理的に無理だと feasibility 項（control/balance）が落ちて whiff する＝倍率を掛けても低いまま。
DIFFICULTY = {
    "kata": 1.0, "dance": 1.0, "march": 1.0, "bow": 0.9, "squat": 1.15, "backflip": 1.4,
    "karate": 1.1, "kathak": 1.2, "hook": 1.05, "kick": 1.25, "dodge": 0.95,
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


# ---------------------------------------------------------------------------
# ゲーム層: 技難度つきラウンド → best-of-N マッチ → 単純トーナメント。
# ---------------------------------------------------------------------------

@dataclass
class Move:
    """1 ファイターの 1 技の結果（style 倍率込みのラウンド得点 + 描画用 kps）。"""

    move: str
    score: float                      # min(100, overall * difficulty)
    card: Scorecard
    kps: np.ndarray = field(repr=False, default=None)
    fps: float = 30.0


def evaluate(robot: str, move: str, *, sim: bool = False) -> Move:
    """robot が move を実行した時の難度込みラウンド得点を返す（相手に依らず決定的）。"""
    from robotdance_retarget.kinematic import retarget
    from robotdance_unitree import get_morphology

    if move not in MOTIONS:
        raise ValueError(f"未知の move '{move}'（利用可能: {sorted(MOTIONS)}）")
    morph = get_morphology(robot)
    mir = MOTIONS[move]()
    mo = retarget(mir, morph)
    cert = None
    if sim:
        from robotdance_sim.backend import certify
        certify(mo, morph)
        cert = mo.sim_certificate
    card = score_fighter(mo.retarget_metrics or {}, cert)
    score = round(min(100.0, card.overall * DIFFICULTY.get(move, 1.0)), 1)
    return Move(move, score, card, mo.keypoints_3d_array(), float(mir.fps))


@dataclass
class RoundResult:
    move: str
    p1_score: float
    p2_score: float
    winner: str          # robot 名 / "TIE"


@dataclass
class MatchResult:
    p1: str
    p2: str
    rounds: list[RoundResult]
    p1_rounds: int
    p2_rounds: int
    p1_total: float
    p2_total: float
    winner: str
    # 描画用ハイライト（最も差がついたラウンドの両者 kps）。
    hi_move: str = ""
    p1_kps: np.ndarray = field(repr=False, default=None)
    p2_kps: np.ndarray = field(repr=False, default=None)
    fps: float = 30.0


def play_match(p1: str, p2: str, moves: list[str], *, sim: bool = False,
               _cache: dict | None = None) -> MatchResult:
    """best-of-N マッチ: 各 move で両者を採点しラウンド勝者を決め、勝ちラウンド数→総得点で勝敗。

    _cache は (robot, move)→Move の評価キャッシュ（トーナメントで再計算を避ける）。
    """
    cache = _cache if _cache is not None else {}

    def ev(robot: str, move: str) -> Move:
        key = (robot, move)
        if key not in cache:
            cache[key] = evaluate(robot, move, sim=sim)
        return cache[key]

    rounds: list[RoundResult] = []
    p1w = p2w = 0
    p1t = p2t = 0.0
    best_margin = -1.0
    hi = (moves[0] if moves else "kata", None, None, 30.0)
    for mv in moves:
        m1, m2 = ev(p1, mv), ev(p2, mv)
        if m1.score > m2.score:
            rw, p1w = p1, p1w + 1
        elif m2.score > m1.score:
            rw, p2w = p2, p2w + 1
        else:
            rw = "TIE"
        rounds.append(RoundResult(mv, m1.score, m2.score, rw))
        p1t += m1.score
        p2t += m2.score
        if abs(m1.score - m2.score) >= best_margin:
            best_margin = abs(m1.score - m2.score)
            hi = (mv, m1.kps, m2.kps, m1.fps)
    if p1w > p2w or (p1w == p2w and p1t > p2t):
        winner = p1
    elif p2w > p1w or (p2w == p1w and p2t > p1t):
        winner = p2
    else:
        winner = "DRAW"
    return MatchResult(p1, p2, rounds, p1w, p2w, round(p1t, 1), round(p2t, 1), winner,
                       hi_move=hi[0], p1_kps=hi[1], p2_kps=hi[2], fps=hi[3])


@dataclass
class TournamentResult:
    bracket: list[list[MatchResult]]   # ラウンドごとの試合（[準々/準決/決勝...]）
    champion: str
    final: MatchResult
    byes: list[str] = field(default_factory=list)


def run_tournament(robots: list[str], moves: list[str], *, sim: bool = False) -> TournamentResult:
    """単純な単欠トーナメント（入力順シード）。各試合は best-of-N マッチ。チャンピオンを返す。

    奇数なら先頭が bye で次へ進む。評価はキャッシュ共有で (robot, move) を一度だけ計算。
    """
    if len(robots) < 2:
        raise ValueError("トーナメントには 2 体以上必要です")
    cache: dict = {}
    alive = list(robots)
    bracket: list[list[MatchResult]] = []
    byes: list[str] = []
    final: MatchResult | None = None
    while len(alive) > 1:
        nxt: list[str] = []
        rnd: list[MatchResult] = []
        i = 0
        if len(alive) % 2 == 1:           # 奇数: 先頭が bye
            byes.append(alive[0])
            nxt.append(alive[0])
            i = 1
        while i < len(alive):
            a, b = alive[i], alive[i + 1]
            m = play_match(a, b, moves, sim=sim, _cache=cache)
            rnd.append(m)
            nxt.append(m.winner if m.winner != "DRAW" else a)  # DRAW は上シード(a)勝ち抜け
            i += 2
        bracket.append(rnd)
        final = rnd[-1]
        alive = nxt
    return TournamentResult(bracket, alive[0], final, byes)


__all__ = [
    "Scorecard", "BattleResult", "Move", "RoundResult", "MatchResult", "TournamentResult",
    "score_fighter", "run_battle", "evaluate", "play_match", "run_tournament",
    "MOTIONS", "DIFFICULTY",
]
