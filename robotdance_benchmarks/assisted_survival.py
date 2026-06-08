"""Assisted survival benchmark — fight motion × robot の PD / RL 物理追従を定量比較。

depth-refine 前後の survival / pose RMSE を並べ、PD-only で失敗した組は RL tracking も試す。
Priority 3 の進捗指標（`demo-fight --assisted` 投入前のゲート判定）。
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from robotdance_motion.fight_refinement import refine_for_fight
from robotdance_retarget.dispatch import retarget_with_backend
from robotdance_retarget.embodiment import RobotMorphology
from robotdance_retarget.gmr_backend import ROBOT_TO_GMR, gmr_available
from robotdance_sim.arena import motion_for_style
from robotdance_sim.assisted_playback import AssistedPlaybackResult, rollout_pd_only
from robotdance_sim.fight_moves import FIGHT_STYLE_NAMES
from robotdance_unitree import get_morphology

_DEFAULT_ROBOTS = (
    "unitree_g1", "unitree_h1", "unitree_h2",
    "booster_t1", "apptronik_apollo", "fourier_n1",
)


@dataclass
class AssistedSurvivalRow:
    robot: str
    style: str
    depth_refine: bool
    survived_frames: int
    total_frames: int
    survival_ratio: float
    mean_pose_rmse: float
    fallen: bool
    retarget_backend: str = "kinematic"  # "kinematic" | "gmr"
    controller: str = "pd"  # "pd" | "rl"
    rl_iterations: int | None = None
    pd_survival: float | None = None  # RL 行のみ: 同一条件の PD baseline


def _fight_reference(
    robot: str,
    style: str,
    *,
    depth_refine: bool,
    duration: float,
    morphology: RobotMorphology,
    retarget_backend: str,
):
    """fight motion → retarget → tracking / assisted 参照。"""
    mir = motion_for_style(style, duration=duration)
    if depth_refine:
        mir = refine_for_fight(mir)
    return retarget_with_backend(mir, morphology, retarget_backend)


def _gmr_supported(robot: str) -> bool:
    return robot in ROBOT_TO_GMR


def evaluate_assisted_survival(
    robot: str,
    style: str,
    *,
    depth_refine: bool = False,
    duration: float = 3.0,
    morphology: Optional[RobotMorphology] = None,
    retarget_backend: str = "kinematic",
) -> AssistedSurvivalRow:
    """1 (robot, style, depth_refine, retarget_backend) の PD-only assisted rollout を評価。"""
    morph = morphology or get_morphology(robot)
    ref = _fight_reference(
        robot, style, depth_refine=depth_refine, duration=duration,
        morphology=morph, retarget_backend=retarget_backend,
    )
    result: AssistedPlaybackResult = rollout_pd_only(ref, morph)
    return AssistedSurvivalRow(
        robot=robot,
        style=style,
        depth_refine=depth_refine,
        survived_frames=result.survived_frames,
        total_frames=result.total_frames,
        survival_ratio=result.survival_ratio,
        mean_pose_rmse=result.mean_pose_rmse,
        fallen=result.fallen,
        retarget_backend=retarget_backend,
        controller="pd",
    )


def evaluate_rl_survival(
    robot: str,
    style: str,
    *,
    depth_refine: bool = False,
    duration: float = 3.0,
    iterations: int = 20,
    steps_per_iter: int = 256,
    morphology: Optional[RobotMorphology] = None,
    pd_survival: float | None = None,
    seed: int = 0,
) -> AssistedSurvivalRow:
    """1 (robot, style) を PPO で学習し RL 物理追従の survival を評価。"""
    from robotdance_models.tracking_policy import train_tracking_policy
    from robotdance_sim.fight_tracking import fight_tracking_reference

    morph = morphology or get_morphology(robot)
    ref = fight_tracking_reference(
        robot, style, depth_refine=depth_refine, duration=duration, morphology=morph,
    )
    policy, _info = train_tracking_policy(
        ref, morph, iterations=iterations, steps_per_iter=steps_per_iter, seed=seed,
    )
    metrics = policy.rollout()[1]
    return AssistedSurvivalRow(
        robot=robot,
        style=style,
        depth_refine=depth_refine,
        survived_frames=int(metrics["survived_frames"]),
        total_frames=int(metrics["reference_frames"]),
        survival_ratio=round(float(metrics["survival_ratio"]), 3),
        mean_pose_rmse=round(float(metrics["mean_pose_rmse"]), 4),
        fallen=bool(metrics.get("fallen", metrics["survival_ratio"] < 1.0)),
        controller="rl",
        rl_iterations=iterations,
        pd_survival=pd_survival,
    )


def run_assisted_survival_benchmark(
    robots: Optional[list[str]] = None,
    styles: Optional[list[str]] = None,
    *,
    duration: float = 3.0,
    compare_refine: bool = True,
    retarget_backends: Optional[list[str]] = None,
    with_rl: bool = False,
    rl_iterations: int = 20,
    rl_only_failures: bool = True,
    rl_seed: int = 0,
) -> dict:
    """robots × styles × retarget_backends の assisted survival を回す。"""
    robots = list(robots or _DEFAULT_ROBOTS)
    styles = list(styles or sorted(FIGHT_STYLE_NAMES))
    backends = list(retarget_backends or ["kinematic"])
    skipped_gmr: list[str] = []
    rows: list[AssistedSurvivalRow] = []
    for backend in backends:
        if backend == "gmr" and not gmr_available():
            raise RuntimeError(
                "retarget backend 'gmr' が未導入です。"
                " git clone https://github.com/YanjieZe/GMR.git && pip install -e GMR/"
            )
        for robot in robots:
            if backend == "gmr" and not _gmr_supported(robot):
                skipped_gmr.append(robot)
                continue
            morph = get_morphology(robot)
            for style in styles:
                rows.append(evaluate_assisted_survival(
                    robot, style, depth_refine=False, duration=duration,
                    morphology=morph, retarget_backend=backend,
                ))
                if compare_refine:
                    rows.append(evaluate_assisted_survival(
                        robot, style, depth_refine=True, duration=duration,
                        morphology=morph, retarget_backend=backend,
                    ))

    rl_rows: list[AssistedSurvivalRow] = []
    if with_rl:
        # RL は kinematic retarget のみ（v0.154）。GMR 比較は PD 列に限定。
        pd_rows = [r for r in rows if r.controller == "pd" and r.retarget_backend == "kinematic"]
        for pd_row in pd_rows:
            if rl_only_failures and pd_row.survival_ratio >= 1.0:
                continue
            morph = get_morphology(pd_row.robot)
            rl_rows.append(evaluate_rl_survival(
                pd_row.robot, pd_row.style,
                depth_refine=pd_row.depth_refine,
                duration=duration,
                iterations=rl_iterations,
                morphology=morph,
                pd_survival=pd_row.survival_ratio,
                seed=rl_seed,
            ))

    all_rows = rows + rl_rows
    pd_rows = [r for r in rows if r.controller == "pd"]
    return {
        "robots": robots,
        "styles": styles,
        "duration": duration,
        "compare_refine": compare_refine,
        "retarget_backends": backends,
        "skipped_gmr_robots": sorted(set(skipped_gmr)),
        "with_rl": with_rl,
        "rl_iterations": rl_iterations if with_rl else None,
        "rl_only_failures": rl_only_failures,
        "rows": [asdict(r) for r in all_rows],
        "rescued": _rescued_pairs(pd_rows),
        "regressed": _regressed_pairs(pd_rows),
        "rescued_by_gmr": _rescued_by_gmr(pd_rows),
        "regressed_by_gmr": _regressed_by_gmr(pd_rows),
        "rescued_by_rl": _rescued_by_rl(rl_rows),
        "rescued_by_rl_only": _rescued_by_rl_only(pd_rows, rl_rows),
    }


def _pair_key(row: AssistedSurvivalRow) -> tuple[str, str, str]:
    return row.robot, row.style, row.retarget_backend


def _by_pair(rows: list[AssistedSurvivalRow]) -> dict[tuple[str, str, str], dict[bool, AssistedSurvivalRow]]:
    out: dict[tuple[str, str, str], dict[bool, AssistedSurvivalRow]] = {}
    for row in rows:
        out.setdefault(_pair_key(row), {})[row.depth_refine] = row
    return out


def _backend_pair_key(row: AssistedSurvivalRow) -> tuple[str, str, bool]:
    return row.robot, row.style, row.depth_refine


def _by_backend_pair(
    rows: list[AssistedSurvivalRow],
) -> dict[tuple[str, str, bool], dict[str, AssistedSurvivalRow]]:
    out: dict[tuple[str, str, bool], dict[str, AssistedSurvivalRow]] = {}
    for row in rows:
        if row.controller != "pd":
            continue
        out.setdefault(_backend_pair_key(row), {})[row.retarget_backend] = row
    return out


def _rescued_by_gmr(rows: list[AssistedSurvivalRow]) -> list[dict]:
    """kinematic より GMR の survival が改善した (robot, style, depth_refine)。"""
    rescued = []
    for key, pair in _by_backend_pair(rows).items():
        kin = pair.get("kinematic")
        gmr = pair.get("gmr")
        if kin is None or gmr is None:
            continue
        if kin.survival_ratio < gmr.survival_ratio:
            rescued.append({
                "robot": key[0],
                "style": key[1],
                "depth_refine": key[2],
                "kin_survival": kin.survival_ratio,
                "gmr_survival": gmr.survival_ratio,
                "delta_survival": round(gmr.survival_ratio - kin.survival_ratio, 3),
            })
    return sorted(rescued, key=lambda x: (-x["delta_survival"], x["robot"], x["style"]))


def _regressed_by_gmr(rows: list[AssistedSurvivalRow]) -> list[dict]:
    """GMR が kinematic より survival を悪化させた組 — 正直に報告。"""
    regressed = []
    for key, pair in _by_backend_pair(rows).items():
        kin = pair.get("kinematic")
        gmr = pair.get("gmr")
        if kin is None or gmr is None:
            continue
        if gmr.survival_ratio < kin.survival_ratio:
            regressed.append({
                "robot": key[0],
                "style": key[1],
                "depth_refine": key[2],
                "kin_survival": kin.survival_ratio,
                "gmr_survival": gmr.survival_ratio,
                "delta_survival": round(gmr.survival_ratio - kin.survival_ratio, 3),
            })
    return sorted(regressed, key=lambda x: (x["delta_survival"], x["robot"], x["style"]))


def _rescued_pairs(rows: list[AssistedSurvivalRow]) -> list[dict]:
    """raw で転倒・早期終了 → refine で survival が改善した (robot, style)。"""
    rescued = []
    for key, pair in _by_pair(rows).items():
        raw = pair.get(False)
        ref = pair.get(True)
        if raw is None or ref is None:
            continue
        if raw.survival_ratio < 1.0 and ref.survival_ratio > raw.survival_ratio:
            rescued.append({
                "robot": key[0],
                "style": key[1],
                "raw_survival": raw.survival_ratio,
                "ref_survival": ref.survival_ratio,
                "delta_survival": round(ref.survival_ratio - raw.survival_ratio, 3),
            })
    return sorted(rescued, key=lambda x: (-x["delta_survival"], x["robot"], x["style"]))


def _regressed_pairs(rows: list[AssistedSurvivalRow]) -> list[dict]:
    """refine で survival が悪化した (robot, style) — 正直に報告。"""
    regressed = []
    for key, pair in _by_pair(rows).items():
        raw = pair.get(False)
        ref = pair.get(True)
        if raw is None or ref is None:
            continue
        if ref.survival_ratio < raw.survival_ratio:
            regressed.append({
                "robot": key[0],
                "style": key[1],
                "raw_survival": raw.survival_ratio,
                "ref_survival": ref.survival_ratio,
                "delta_survival": round(ref.survival_ratio - raw.survival_ratio, 3),
            })
    return sorted(regressed, key=lambda x: (x["delta_survival"], x["robot"], x["style"]))


def _rescued_by_rl(rl_rows: list[AssistedSurvivalRow]) -> list[dict]:
    """PD が失敗 → RL で survival が改善した (robot, style, depth_refine)。"""
    rescued = []
    for row in rl_rows:
        if row.pd_survival is None:
            continue
        if row.pd_survival < 1.0 and row.survival_ratio > row.pd_survival:
            rescued.append({
                "robot": row.robot,
                "style": row.style,
                "depth_refine": row.depth_refine,
                "pd_survival": row.pd_survival,
                "rl_survival": row.survival_ratio,
                "delta_survival": round(row.survival_ratio - row.pd_survival, 3),
            })
    return sorted(rescued, key=lambda x: (-x["delta_survival"], x["robot"], x["style"]))


def _rescued_by_rl_only(
    pd_rows: list[AssistedSurvivalRow],
    rl_rows: list[AssistedSurvivalRow],
) -> list[dict]:
    """depth-refine でも PD<1 のまま → RL で改善（refine では救えなかった組）。"""
    refine_fixed = {
        (r.robot, r.style)
        for r in pd_rows
        if r.depth_refine and r.survival_ratio >= 1.0
    }
    out = []
    for rl in rl_rows:
        if rl.pd_survival is None or rl.pd_survival >= 1.0:
            continue
        if (rl.robot, rl.style) in refine_fixed and not rl.depth_refine:
            continue  # refine で既に救済済み
        if rl.depth_refine:
            continue  # refine 後の PD 失敗は別扱い（rescued_by_rl に入る）
        if rl.survival_ratio > rl.pd_survival:
            out.append({
                "robot": rl.robot,
                "style": rl.style,
                "pd_survival": rl.pd_survival,
                "rl_survival": rl.survival_ratio,
                "delta_survival": round(rl.survival_ratio - rl.pd_survival, 3),
            })
    return sorted(out, key=lambda x: (-x["delta_survival"], x["robot"], x["style"]))


_CSV_COLUMNS = [
    "robot", "style", "depth_refine", "retarget_backend", "controller",
    "survived_frames", "total_frames", "survival_ratio", "mean_pose_rmse", "fallen",
    "rl_iterations", "pd_survival",
]


def write_assisted_survival_csv(report: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=_CSV_COLUMNS)
        writer.writeheader()
        for row in report["rows"]:
            writer.writerow({c: row.get(c) for c in _CSV_COLUMNS})
    return path


def render_assisted_survival_markdown(report: dict) -> str:
    """比較テーブル + rescued / regressed サマリの Markdown。"""
    lines = [
        "# Assisted Survival Benchmark",
        "",
        "PD-only（残差ゼロ）物理追従での fight motion 生存率。",
        "depth-refine = `stabilize_depth` + `balance_depth_refine` を retarget 前に適用。",
        "",
        f"- robots: {', '.join(report['robots'])}",
        f"- styles: {', '.join(report['styles'])}",
        f"- duration: {report['duration']}s（karate/kathak はフィクスチャ長）",
        f"- retarget backends: {', '.join(report.get('retarget_backends', ['kinematic']))}",
    ]
    if report.get("skipped_gmr_robots"):
        lines.append(
            f"- GMR skipped robots: {', '.join(report['skipped_gmr_robots'])}（未対応機種）"
        )
    if report.get("with_rl"):
        lines.append(
            f"- RL: PPO {report['rl_iterations']} iters"
            f"（{'PD 失敗のみ' if report.get('rl_only_failures') else '全組'}）"
        )
    lines.append("")
    backends = report.get("retarget_backends", ["kinematic"])
    if len(backends) > 1 or (len(backends) == 1 and backends[0] != "kinematic"):
        lines += [
            "## Retarget backend comparison",
            "",
            "| robot | style | refine | kin surv | gmr surv | Δ surv | kin RMSE | gmr RMSE |",
            "|-------|-------|--------|----------|----------|--------|----------|----------|",
        ]
        by_backend = {}
        for row in report["rows"]:
            if row.get("controller", "pd") != "pd":
                continue
            key = (row["robot"], row["style"], row["depth_refine"])
            by_backend.setdefault(key, {})[row.get("retarget_backend", "kinematic")] = row
        for (robot, style, refine), pair in sorted(by_backend.items()):
            kin = pair.get("kinematic")
            gmr = pair.get("gmr")
            if not kin or not gmr:
                continue
            delta = gmr["survival_ratio"] - kin["survival_ratio"]
            refine_s = "yes" if refine else "no"
            lines.append(
                f"| {robot} | {style} | {refine_s} | {kin['survival_ratio']:.3f} | "
                f"{gmr['survival_ratio']:.3f} | {delta:+.3f} | "
                f"{kin['mean_pose_rmse']:.3f} | {gmr['mean_pose_rmse']:.3f} |"
            )
        lines.append("")
        if report.get("rescued_by_gmr"):
            lines += ["## Rescued by GMR (vs kinematic)", ""]
            for r in report["rescued_by_gmr"]:
                refine = "refine" if r["depth_refine"] else "raw"
                lines.append(
                    f"- **{r['robot']} / {r['style']} ({refine})**: "
                    f"kin {r['kin_survival']:.3f} → gmr {r['gmr_survival']:.3f} "
                    f"(Δ {r['delta_survival']:+.3f})"
                )
            lines.append("")
        if report.get("regressed_by_gmr"):
            lines += ["## Regressed by GMR (honest)", ""]
            for r in report["regressed_by_gmr"]:
                refine = "refine" if r["depth_refine"] else "raw"
                lines.append(
                    f"- **{r['robot']} / {r['style']} ({refine})**: "
                    f"kin {r['kin_survival']:.3f} → gmr {r['gmr_survival']:.3f} "
                    f"(Δ {r['delta_survival']:+.3f})"
                )
            lines.append("")
    if report["compare_refine"]:
        for backend in backends:
            lines += [
                f"## Raw vs depth-refine ({backend})",
                "",
                "| robot | style | raw surv | ref surv | Δ surv | raw RMSE | ref RMSE |",
                "|-------|-------|----------|----------|--------|----------|----------|",
            ]
            by_pair = {}
            for row in report["rows"]:
                if row.get("controller", "pd") != "pd":
                    continue
                if row.get("retarget_backend", "kinematic") != backend:
                    continue
                by_pair.setdefault((row["robot"], row["style"]), {})[row["depth_refine"]] = row
            for (robot, style), pair in sorted(by_pair.items()):
                raw = pair.get(False, {})
                ref = pair.get(True, {})
                if not raw or not ref:
                    continue
                delta = ref["survival_ratio"] - raw["survival_ratio"]
                lines.append(
                    f"| {robot} | {style} | {raw['survival_ratio']:.3f} | "
                    f"{ref['survival_ratio']:.3f} | {delta:+.3f} | "
                    f"{raw['mean_pose_rmse']:.3f} | {ref['mean_pose_rmse']:.3f} |"
                )
            lines.append("")
        if report["rescued"]:
            lines += ["## Rescued by depth-refine", ""]
            for r in report["rescued"]:
                lines.append(
                    f"- **{r['robot']} / {r['style']}**: "
                    f"{r['raw_survival']:.3f} → {r['ref_survival']:.3f} "
                    f"(Δ {r['delta_survival']:+.3f})"
                )
            lines.append("")
        if report["regressed"]:
            lines += ["## Regressed (honest)", ""]
            for r in report["regressed"]:
                lines.append(
                    f"- **{r['robot']} / {r['style']}**: "
                    f"{r['raw_survival']:.3f} → {r['ref_survival']:.3f} "
                    f"(Δ {r['delta_survival']:+.3f})"
                )
            lines.append("")
    if report.get("with_rl") and any(r.get("controller") == "rl" for r in report["rows"]):
        lines += [
            "## PD vs RL（PD が失敗した組のみ）",
            "",
            "| robot | style | refine | PD surv | RL surv | Δ surv | PD RMSE | RL RMSE |",
            "|-------|-------|--------|---------|---------|--------|---------|---------|",
        ]
        for row in report["rows"]:
            if row.get("controller") != "rl":
                continue
            pd_row = next(
                (r for r in report["rows"]
                 if r.get("controller") == "pd"
                 and r["robot"] == row["robot"]
                 and r["style"] == row["style"]
                 and r["depth_refine"] == row["depth_refine"]
                 and r.get("retarget_backend", "kinematic") == "kinematic"),
                None,
            )
            pd_surv = row.get("pd_survival") if row.get("pd_survival") is not None else (
                pd_row["survival_ratio"] if pd_row else float("nan")
            )
            pd_rmse = pd_row["mean_pose_rmse"] if pd_row else float("nan")
            delta = row["survival_ratio"] - pd_surv
            refine = "yes" if row["depth_refine"] else "no"
            lines.append(
                f"| {row['robot']} | {row['style']} | {refine} | {pd_surv:.3f} | "
                f"{row['survival_ratio']:.3f} | {delta:+.3f} | {pd_rmse:.3f} | "
                f"{row['mean_pose_rmse']:.3f} |"
            )
        lines.append("")
        if report.get("rescued_by_rl"):
            lines += ["## Rescued by RL", ""]
            for r in report["rescued_by_rl"]:
                refine = "refine" if r["depth_refine"] else "raw"
                lines.append(
                    f"- **{r['robot']} / {r['style']} ({refine})**: "
                    f"PD {r['pd_survival']:.3f} → RL {r['rl_survival']:.3f} "
                    f"(Δ {r['delta_survival']:+.3f})"
                )
            lines.append("")
        if report.get("rescued_by_rl_only"):
            lines += ["## Rescued by RL only (refine でも未救済)", ""]
            for r in report["rescued_by_rl_only"]:
                lines.append(
                    f"- **{r['robot']} / {r['style']}**: "
                    f"PD raw {r['pd_survival']:.3f} → RL {r['rl_survival']:.3f} "
                    f"(Δ {r['delta_survival']:+.3f})"
                )
            lines.append("")
    if not report["compare_refine"]:
        lines += [
            "## Results",
            "",
            "| robot | style | survival | RMSE | fallen |",
            "|-------|-------|----------|------|--------|",
        ]
        for row in report["rows"]:
            lines.append(
                f"| {row['robot']} | {row['style']} | {row['survival_ratio']:.3f} | "
                f"{row['mean_pose_rmse']:.3f} | {row['fallen']} |"
            )
        lines.append("")
    note = "> ⚠️ v0 baseline: PD-only / 単参照 PPO tracking。"
    if report.get("with_rl"):
        note += " RL は PD 失敗組への追加評価。"
    note += " 真の 2 体接触スパーリングは未対応。"
    lines.append(note)
    return "\n".join(lines) + "\n"


__all__ = [
    "AssistedSurvivalRow",
    "evaluate_assisted_survival",
    "evaluate_rl_survival",
    "render_assisted_survival_markdown",
    "run_assisted_survival_benchmark",
    "write_assisted_survival_csv",
]
