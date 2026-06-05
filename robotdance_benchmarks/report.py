"""benchmark 結果の CSV / Markdown leaderboard 出力（v0）。"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

_COLUMNS = [
    "motion_id", "motion_class", "robot", "height_scale", "bone_direction_cosine",
    "foot_sliding", "joint_flexion_violation", "verdict", "airborne_ratio",
    "balance_violation_ratio", "torque_ratio", "gravity_torque_nm",
    "dynamic_torque_nm", "max_joint_ang_speed",
    "source_confidence", "jitter",
]


def write_csv(report: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=_COLUMNS)
        writer.writeheader()
        for row in report["rows"]:
            writer.writerow({c: row.get(c) for c in _COLUMNS})
    return path


def _mean(values: list[Any]) -> float | None:
    nums = [v for v in values if isinstance(v, (int, float))]
    return round(sum(nums) / len(nums), 4) if nums else None


def aggregate_by_robot(report: dict) -> list[dict]:
    """ロボットごとの集計（pass_rate / 平均 retarget 指標）。"""
    out = []
    for robot in report["robots"]:
        rows = [r for r in report["rows"] if r["robot"] == robot]
        verdicts = [r["verdict"] for r in rows if r["verdict"]]
        passed = sum(1 for v in verdicts if v == "PASS")
        out.append({
            "robot": robot,
            "n": len(rows),
            "pass_rate": round(passed / len(verdicts), 3) if verdicts else None,
            "mean_bone_dir_cos": _mean([r["bone_direction_cosine"] for r in rows]),
            "mean_foot_sliding": _mean([r["foot_sliding"] for r in rows]),
            "mean_height_scale": _mean([r["height_scale"] for r in rows]),
            "mean_flexion_violation": _mean([r.get("joint_flexion_violation") for r in rows]),
            "mean_dynamic_torque_nm": _mean([r.get("dynamic_torque_nm") for r in rows]),
        })
    return out


def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)


def write_markdown(report: dict, path: str | Path, *, title: str = "RobotDance Benchmark") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"# {title}",
        "",
        f"motions: **{len(report['motions'])}** × robots: **{len(report['robots'])}** "
        f"= {len(report['rows'])} runs · sim: **{'on' if report['sim_available'] else 'off (mujoco 無し)'}**",
        "",
        "> ⚠️ v0: 近似形態プロキシ。sim は実 URDF 慣性テンソルで検証（v0.52）。実機保証ではない（各 README 参照）。",
        "",
        "## Leaderboard（robot 別集計）",
        "",
        "| robot | runs | PASS率 | 平均 bone方向cos | 平均 foot_sliding | 平均 height_scale | "
        "平均 屈曲違反率 | 平均 動的tq(N·m) |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for a in aggregate_by_robot(report):
        lines.append(
            f"| {a['robot']} | {a['n']} | {_fmt(a['pass_rate'])} | {_fmt(a['mean_bone_dir_cos'])} | "
            f"{_fmt(a['mean_foot_sliding'])} | {_fmt(a['mean_height_scale'])} | "
            f"{_fmt(a.get('mean_flexion_violation'))} | {_fmt(a.get('mean_dynamic_torque_nm'))} |"
        )

    lines += [
        "",
        "## 全 run（motion × robot）",
        "",
        "> `torque×` = 動的tq / 実 per-joint effort 上限の最大（>1.0 で REJECT）。`重力tq` は重力保持（準静的）",
        "> 成分、`動的tq` は重力＋並進＋回転慣性の合計（v0.62/v0.63）。両者の差が**慣性寄与**で、速い運動ほど開く。",
        "",
        "| motion | class | robot | verdict | airborne | balance | torque× | 重力tq | 動的tq | 角速度 | "
        "foot_slide | bone_cos | 屈曲違反 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in report["rows"]:
        lines.append(
            f"| {r['motion_id']} | {r['motion_class']} | {r['robot']} | {_fmt(r['verdict'])} | "
            f"{_fmt(r['airborne_ratio'])} | {_fmt(r['balance_violation_ratio'])} | "
            f"{_fmt(r['torque_ratio'])} | {_fmt(r.get('gravity_torque_nm'))} | "
            f"{_fmt(r.get('dynamic_torque_nm'))} | {_fmt(r['max_joint_ang_speed'])} | "
            f"{_fmt(r['foot_sliding'])} | {_fmt(r['bone_direction_cosine'])} | "
            f"{_fmt(r.get('joint_flexion_violation'))} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
