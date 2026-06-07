"""benchmark 結果の CSV / Markdown leaderboard 出力（v0）。"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Optional

_COLUMNS = [
    "motion_id", "motion_class", "robot", "height_scale", "bone_direction_cosine",
    "foot_sliding", "endeffector_reach_error", "joint_flexion_violation", "verdict", "airborne_ratio",
    "balance_violation_ratio", "torque_ratio", "gravity_torque_nm",
    "dynamic_torque_nm", "max_joint_ang_speed", "binding_axis", "binding_util",
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
            "mean_endeffector_reach_error": _mean([r.get("endeffector_reach_error") for r in rows]),
            "mean_height_scale": _mean([r["height_scale"] for r in rows]),
            "mean_flexion_violation": _mean([r.get("joint_flexion_violation") for r in rows]),
            "mean_dynamic_torque_nm": _mean([r.get("dynamic_torque_nm") for r in rows]),
            "top_binding_axis": _mode([r.get("binding_axis") for r in rows]),
        })
    return out


def aggregate_by_motion(report: dict) -> list[dict]:
    """motion ごとの集計（何機種が実行可能か / 平均 torque・balance / 最頻 律速軸）。

    robot 別集計（aggregate_by_robot）の双対。**全機種で REJECT な動作（本質的に難しい）**と
    **機種依存で可否が割れる動作**を見分けるための行。pass_rate は機種を跨いだ PASS 率。
    """
    out = []
    for motion in report["motions"]:
        rows = [r for r in report["rows"] if r["motion_id"] == motion]
        verdicts = [r["verdict"] for r in rows if r["verdict"]]
        passed = sum(1 for v in verdicts if v == "PASS")
        out.append({
            "motion_id": motion,
            "motion_class": rows[0]["motion_class"] if rows else None,
            "n_robots": len(rows),
            "pass_rate": round(passed / len(verdicts), 3) if verdicts else None,
            "mean_endeffector_reach_error": _mean([r.get("endeffector_reach_error") for r in rows]),
            "mean_torque_ratio": _mean([r.get("torque_ratio") for r in rows]),
            "mean_balance_violation": _mean([r.get("balance_violation_ratio") for r in rows]),
            "top_binding_axis": _mode([r.get("binding_axis") for r in rows]),
        })
    return out


def _mode(values: list[Any]) -> Optional[str]:
    """最頻値（機種の系統的な弱点軸）。None は除外。同数は最初に出た軸。"""
    counts: dict[str, int] = {}
    for v in values:
        if v:
            counts[v] = counts.get(v, 0) + 1
    return max(counts, key=counts.get) if counts else None


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
        "平均 屈曲違反率 | 平均 動的tq(N·m) | 最頻 律速軸 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for a in aggregate_by_robot(report):
        lines.append(
            f"| {a['robot']} | {a['n']} | {_fmt(a['pass_rate'])} | {_fmt(a['mean_bone_dir_cos'])} | "
            f"{_fmt(a['mean_foot_sliding'])} | {_fmt(a['mean_height_scale'])} | "
            f"{_fmt(a.get('mean_flexion_violation'))} | {_fmt(a.get('mean_dynamic_torque_nm'))} | "
            f"{_fmt(a.get('top_binding_axis'))} |"
        )

    lines += [
        "",
        "## Leaderboard（motion 別集計）",
        "",
        "> pass率=機種を跨いだ PASS 率。低い動作ほど**どの機種でも難しい**（本質的難度）。",
        "",
        "| motion | class | robots | PASS率 | 平均 torque× | 平均 balance違反 | 最頻 律速軸 |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for a in aggregate_by_motion(report):
        lines.append(
            f"| {a['motion_id']} | {_fmt(a['motion_class'])} | {a['n_robots']} | "
            f"{_fmt(a['pass_rate'])} | {_fmt(a['mean_torque_ratio'])} | "
            f"{_fmt(a['mean_balance_violation'])} | {_fmt(a['top_binding_axis'])} |"
        )

    lines += [
        "",
        "## 全 run（motion × robot）",
        "",
        "> `torque×` = 動的tq / 実 per-joint effort 上限の最大（>1.0 で REJECT）。`重力tq` は重力保持（準静的）",
        "> 成分、`動的tq` は重力＋並進＋回転慣性の合計（v0.62/v0.63）。両者の差が**慣性寄与**で、速い運動ほど開く。",
        "",
        "| motion | class | robot | verdict | 律速軸(util) | airborne | balance | torque× | 重力tq | "
        "動的tq | 角速度 | foot_slide | bone_cos | 屈曲違反 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in report["rows"]:
        ba = r.get("binding_axis")
        bind = f"{ba} ({r['binding_util']:.2f})" if ba and r.get("binding_util") is not None else "—"
        lines.append(
            f"| {r['motion_id']} | {r['motion_class']} | {r['robot']} | {_fmt(r['verdict'])} | "
            f"{bind} | {_fmt(r['airborne_ratio'])} | {_fmt(r['balance_violation_ratio'])} | "
            f"{_fmt(r['torque_ratio'])} | {_fmt(r.get('gravity_torque_nm'))} | "
            f"{_fmt(r.get('dynamic_torque_nm'))} | {_fmt(r['max_joint_ang_speed'])} | "
            f"{_fmt(r['foot_sliding'])} | {_fmt(r['bone_direction_cosine'])} | "
            f"{_fmt(r.get('joint_flexion_violation'))} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
