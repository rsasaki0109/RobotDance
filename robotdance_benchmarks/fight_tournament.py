"""Physical HumanoidBattle — トーナメント各試合を `demo-fight` のヒット採点で決める。

型（kata）の実行品質採点（`battle.run_tournament`）ではなく、MuJoCo arena で実際に殴り合わせ、
ラウンド／試合／ブラケットの勝者をヒット数で決定する。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robotdance_sim.arena import FightResult

from robotdance_sim.fight_moves import FIGHT_STYLE_NAMES as FIGHT_STYLES


@dataclass
class FightRoundResult:
    """1 ラウンド = 1 fight style の bout 結果。"""

    style: str
    p1_hits: int
    p2_hits: int
    winner: str          # robot 名 / "TIE"


@dataclass
class FightMatchResult:
    p1: str
    p2: str
    rounds: list[FightRoundResult]
    p1_rounds: int       # 勝ったラウンド数
    p2_rounds: int
    p1_total_hits: int   # 全ラウンドのヒット合計（タイブレーク用）
    p2_total_hits: int
    winner: str
    hi_style: str = ""   # ハイライト用（差が最大のラウンド style）
    hi_fight: FightResult | None = field(repr=False, default=None)


@dataclass
class FightTournamentResult:
    bracket: list[list[FightMatchResult]]
    champion: str
    final: FightMatchResult
    byes: list[str] = field(default_factory=list)


def resolve_assisted_corner(
    assisted: str | None,
    *,
    champion: str,
    p1: str,
    p2: str,
) -> str | None:
    """`assisted='champion'` を決勝の p1/p2 コーナーへ解決する。"""
    if assisted is None:
        return None
    if assisted == "champion":
        if champion == p1:
            return "p1"
        if champion == p2:
            return "p2"
        return "p1"  # DRAW 時は p1 にフォールバック
    return assisted


def _run_bout(p1: str, p2: str, style: str, *, duration: float, separation: float,
              render: bool, mesh: bool, urdf_a: str | None, urdf_b: str | None):
    import mujoco  # noqa: F401 — arena が MuJoCo 前提

    from robotdance_sim.arena import run_fight
    from robotdance_unitree import get_morphology

    return run_fight(
        get_morphology(p1), get_morphology(p2),
        name_a=p1, name_b=p2, duration=duration, separation=separation,
        style=style, render=render, mesh=mesh,
        urdf_a=urdf_a, urdf_b=urdf_b,
    )


def play_fight_match(
    p1: str,
    p2: str,
    styles: list[str],
    *,
    duration: float = 4.0,
    separation: float = 0.17,
    render: bool = False,
    mesh: bool = False,
    urdf_a: str | None = None,
    urdf_b: str | None = None,
) -> FightMatchResult:
    """best-of-N fight マッチ: 各 style で bout を行い、ラウンド勝者→総ヒットで勝敗。"""
    bad = [s for s in styles if s not in FIGHT_STYLES]
    if bad:
        raise ValueError(
            f"physical トーナメントの style は {sorted(FIGHT_STYLES)} のみ"
            f"（未知: {bad}）"
        )

    rounds: list[FightRoundResult] = []
    p1w = p2w = 0
    p1th = p2th = 0
    best_margin = -1
    hi_style = styles[0] if styles else "boxing"
    hi_fight = None

    for style in styles:
        res = _run_bout(p1, p2, style, duration=duration, separation=separation,
                        render=render, mesh=mesh, urdf_a=urdf_a, urdf_b=urdf_b)
        if res.p1_hits > res.p2_hits:
            rw, p1w = p1, p1w + 1
        elif res.p2_hits > res.p1_hits:
            rw, p2w = p2, p2w + 1
        else:
            rw = "TIE"
        rounds.append(FightRoundResult(style, res.p1_hits, res.p2_hits, rw))
        p1th += res.p1_hits
        p2th += res.p2_hits
        margin = abs(res.p1_hits - res.p2_hits)
        if margin >= best_margin:
            best_margin = margin
            hi_style = style
            hi_fight = res

    if p1w > p2w or (p1w == p2w and p1th > p2th):
        winner = p1
    elif p2w > p1w or (p2w == p1w and p2th > p1th):
        winner = p2
    else:
        winner = "DRAW"
    return FightMatchResult(
        p1, p2, rounds, p1w, p2w, p1th, p2th, winner,
        hi_style=hi_style, hi_fight=hi_fight,
    )


def run_fight_tournament(
    robots: list[str],
    styles: list[str],
    *,
    duration: float = 4.0,
    separation: float = 0.17,
    mesh: bool = False,
    urdf_a: str | None = None,
    urdf_b: str | None = None,
) -> FightTournamentResult:
    """単欠トーナメント。各試合は `play_fight_match`（ヒット採点）。"""
    if len(robots) < 2:
        raise ValueError("トーナメントには 2 体以上必要です")

    alive = list(robots)
    bracket: list[list[FightMatchResult]] = []
    byes: list[str] = []
    final: FightMatchResult | None = None

    while len(alive) > 1:
        nxt: list[str] = []
        rnd: list[FightMatchResult] = []
        i = 0
        if len(alive) % 2 == 1:
            byes.append(alive[0])
            nxt.append(alive[0])
            i = 1
        while i < len(alive):
            a, b = alive[i], alive[i + 1]
            m = play_fight_match(
                a, b, styles, duration=duration, separation=separation,
                mesh=mesh, urdf_a=urdf_a, urdf_b=urdf_b,
            )
            rnd.append(m)
            nxt.append(m.winner if m.winner != "DRAW" else a)
            i += 2
        bracket.append(rnd)
        final = rnd[-1]
        alive = nxt
    return FightTournamentResult(bracket, alive[0], final, byes)


__all__ = [
    "FIGHT_STYLES",
    "FightRoundResult",
    "FightMatchResult",
    "FightTournamentResult",
    "play_fight_match",
    "resolve_assisted_corner",
    "run_fight_tournament",
]
