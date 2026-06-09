"""Geometric vs contact fight 採点の比較 benchmark。

1 回の PD sparring（contact_scoring=True）から幾何ヒットと接触力ヒットを同時取得し、
勝者一致率・ヒット差分を報告する。
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from robotdance_retarget.dispatch import check_retarget_backend_for_robots
from robotdance_retarget.embodiment import RobotMorphology
from robotdance_retarget.gmr_backend import ROBOT_TO_GMR, gmr_available
from robotdance_sim.arena import run_fight
from robotdance_sim.fight_moves import FIGHT_STYLE_NAMES
from robotdance_unitree import get_morphology

_DEFAULT_ROBOTS = (
    "unitree_g1", "unitree_h1", "unitree_h2",
    "booster_t1", "apptronik_apollo", "fourier_n1",
)
_DEFAULT_OPPONENT = "unitree_h1"


@dataclass
class FightScoringCompareRow:
    p1: str
    p2: str
    style: str
    depth_refine: bool
    geom_p1_hits: int
    geom_p2_hits: int
    contact_p1_hits: int
    contact_p2_hits: int
    geom_winner: str
    contact_winner: str
    winner_agrees: bool
    delta_p1_hits: int
    delta_p2_hits: int
    retarget_backend: str = "kinematic"


def _gmr_supported(robot: str) -> bool:
    return robot in ROBOT_TO_GMR


def _winner_from_hits(
    p1: str,
    p2: str,
    morph_a: RobotMorphology,
    morph_b: RobotMorphology,
    p1_hits: int,
    p2_hits: int,
) -> str:
    if p1_hits > p2_hits:
        return p1
    if p2_hits > p1_hits:
        return p2
    ha, hb = morph_a.nominal_height, morph_b.nominal_height
    if ha > hb:
        return p1
    if hb > ha:
        return p2
    return "DRAW"


def evaluate_fight_scoring_compare(
    p1: str,
    p2: str,
    style: str,
    *,
    depth_refine: bool = False,
    duration: float = 3.0,
    separation: float = 0.17,
    retarget_backend: str = "kinematic",
) -> FightScoringCompareRow:
    """1 bout を contact_scoring で走らせ、幾何 vs 接触のヒットを比較。"""
    check_retarget_backend_for_robots([p1, p2], retarget_backend)
    morph_a = get_morphology(p1)
    morph_b = get_morphology(p2)
    res = run_fight(
        morph_a, morph_b,
        name_a=p1, name_b=p2, duration=duration, separation=separation,
        style=style, render=False, depth_refine=depth_refine,
        retarget_backend=retarget_backend, sparring=True, contact_scoring=True,
    )
    assert res.scoring_mode == "contact"
    g1 = res.p1_geom_hits if res.p1_geom_hits is not None else 0
    g2 = res.p2_geom_hits if res.p2_geom_hits is not None else 0
    c1, c2 = res.p1_hits, res.p2_hits
    geom_w = _winner_from_hits(p1, p2, morph_a, morph_b, g1, g2)
    contact_w = res.winner
    return FightScoringCompareRow(
        p1=p1,
        p2=p2,
        style=style,
        depth_refine=depth_refine,
        geom_p1_hits=g1,
        geom_p2_hits=g2,
        contact_p1_hits=c1,
        contact_p2_hits=c2,
        geom_winner=geom_w,
        contact_winner=contact_w,
        winner_agrees=geom_w == contact_w,
        delta_p1_hits=c1 - g1,
        delta_p2_hits=c2 - g2,
        retarget_backend=retarget_backend,
    )


def run_fight_scoring_compare_benchmark(
    robots: Optional[list[str]] = None,
    opponent: str = _DEFAULT_OPPONENT,
    styles: Optional[list[str]] = None,
    *,
    duration: float = 3.0,
    separation: float = 0.17,
    compare_refine: bool = True,
    retarget_backends: Optional[list[str]] = None,
) -> dict:
    robots = list(robots or _DEFAULT_ROBOTS)
    styles = list(styles or sorted(FIGHT_STYLE_NAMES))
    backends = list(retarget_backends or ["kinematic"])
    skipped_gmr: list[str] = []
    rows: list[FightScoringCompareRow] = []

    for backend in backends:
        if backend == "gmr" and not gmr_available():
            raise RuntimeError(
                "retarget backend 'gmr' が未導入です。"
                " git clone https://github.com/YanjieZe/GMR.git && pip install -e GMR/"
            )
        for p1 in robots:
            if p1 == opponent:
                continue
            if backend == "gmr" and (
                not _gmr_supported(p1) or not _gmr_supported(opponent)
            ):
                skipped_gmr.append(p1)
                continue
            for style in styles:
                rows.append(evaluate_fight_scoring_compare(
                    p1, opponent, style, depth_refine=False,
                    duration=duration, separation=separation,
                    retarget_backend=backend,
                ))
                if compare_refine:
                    rows.append(evaluate_fight_scoring_compare(
                        p1, opponent, style, depth_refine=True,
                        duration=duration, separation=separation,
                        retarget_backend=backend,
                    ))

    agrees = sum(1 for r in rows if r.winner_agrees)
    disagrees = [asdict(r) for r in rows if not r.winner_agrees]
    return {
        "robots": robots,
        "opponent": opponent,
        "styles": styles,
        "duration": duration,
        "separation": separation,
        "compare_refine": compare_refine,
        "retarget_backends": backends,
        "skipped_gmr_robots": sorted(set(skipped_gmr)),
        "rows": [asdict(r) for r in rows],
        "winner_agreement_rate": round(agrees / max(len(rows), 1), 3),
        "winner_disagreements": disagrees,
    }


_CSV_COLUMNS = [
    "p1", "p2", "style", "depth_refine", "retarget_backend",
    "geom_p1_hits", "geom_p2_hits", "contact_p1_hits", "contact_p2_hits",
    "delta_p1_hits", "delta_p2_hits",
    "geom_winner", "contact_winner", "winner_agrees",
]


def write_fight_scoring_compare_csv(report: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=_CSV_COLUMNS)
        writer.writeheader()
        for row in report["rows"]:
            writer.writerow({c: row.get(c) for c in _CSV_COLUMNS})
    return path


def render_fight_scoring_compare_markdown(report: dict) -> str:
    lines = [
        "# Fight Scoring Compare Benchmark",
        "",
        "同一 PD sparring bout で幾何ヒット vs MuJoCo 接触力ヒットを比較。",
        "",
        f"- p1 robots: {', '.join(report['robots'])}",
        f"- opponent (p2): {report['opponent']}",
        f"- styles: {', '.join(report['styles'])}",
        f"- duration: {report['duration']}s",
        f"- separation: {report['separation']}m",
        f"- retarget backends: {', '.join(report.get('retarget_backends', ['kinematic']))}",
        f"- winner agreement: **{report['winner_agreement_rate']:.1%}** "
        f"({sum(1 for r in report['rows'] if r['winner_agrees'])}/{len(report['rows'])} bouts)",
        "",
    ]
    if report.get("skipped_gmr_robots"):
        lines.append(
            f"- GMR skipped p1: {', '.join(report['skipped_gmr_robots'])}"
        )
        lines.append("")

    for backend in report.get("retarget_backends", ["kinematic"]):
        lines += [
            f"## Geometric vs contact ({backend})",
            "",
            "| p1 | style | refine | geom | contact | Δp1 | Δp2 | geom W | contact W | agree |",
            "|----|-------|--------|------|---------|-----|-----|--------|-----------|-------|",
        ]
        for row in report["rows"]:
            if row.get("retarget_backend", "kinematic") != backend:
                continue
            refine = "yes" if row["depth_refine"] else "no"
            agree = "✓" if row["winner_agrees"] else "✗"
            lines.append(
                f"| {row['p1']} | {row['style']} | {refine} | "
                f"{row['geom_p1_hits']}-{row['geom_p2_hits']} | "
                f"{row['contact_p1_hits']}-{row['contact_p2_hits']} | "
                f"{row['delta_p1_hits']:+d} | {row['delta_p2_hits']:+d} | "
                f"{row['geom_winner']} | {row['contact_winner']} | {agree} |"
            )
        lines.append("")

    if report["winner_disagreements"]:
        lines += ["## Winner disagreements", ""]
        for row in report["winner_disagreements"]:
            refine = "refine" if row["depth_refine"] else "raw"
            lines.append(
                f"- **{row['p1']} vs {row['p2']} / {row['style']} ({refine})**: "
                f"geom {row['geom_p1_hits']}-{row['geom_p2_hits']} → {row['geom_winner']} / "
                f"contact {row['contact_p1_hits']}-{row['contact_p2_hits']} → {row['contact_winner']}"
            )
        lines.append("")

    lines.append(
        "> ⚠️ contact 採点は sparring（PD 物理）専用。"
        " ブラケット ELO は幾何のまま。"
    )
    return "\n".join(lines) + "\n"


__all__ = [
    "FightScoringCompareRow",
    "evaluate_fight_scoring_compare",
    "render_fight_scoring_compare_markdown",
    "run_fight_scoring_compare_benchmark",
    "write_fight_scoring_compare_csv",
]
