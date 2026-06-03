"""Model / Motion Card 生成（§7, v0）。

RobotDance の成果物（RD-MIR / RD-Motion）から、責任ある公開・利用に必要な情報を構造化した
**カード**を生成する: data lineage（source→extractor→retarget→sim→policy の連鎖）・
license composition・**failure modes（既知の v0 限界）**・safety limits・metrics。

dataset 全体の license firewall 内訳は別途 `robotdance_data` の Data Bill of Materials
（DATA_CARD.md）が担う。本モジュールは **個別 artifact の説明責任**（どう作られ・何に使え・
何に使えないか・どこで壊れるか）を、機械可読 dict と Markdown の両方で提供する。

⚠️ v0: failure modes は artifact が使った手法（extractor / retarget / sim backend / control_mode）
から curated registry を引いて組み立てる。網羅ではなく「正直に既知の限界を明示する」ことが目的。
"""

from __future__ import annotations

from typing import Any, Optional

from .rd_mir import RdMir
from .rd_motion import RdMotion

CARD_VERSION = "0"

# license_state → 再配布/商用の可否と注意。
_LICENSE_INFO: dict[str, dict[str, Any]] = {
    "redistributable": {"redistribution": True, "commercial": True,
                        "note": "派生 motion を再配布可。"},
    "trainable": {"redistribution": False, "commercial": False,
                  "note": "学習利用可だが派生物の再配布条件は要確認。"},
    "commercial_allowed": {"redistribution": False, "commercial": True,
                           "note": "商用利用可。再配布条件は source に従う。"},
    "research_only": {"redistribution": False, "commercial": False,
                      "note": "研究用途のみ。商用・再配布不可。"},
    "unknown": {"redistribution": False, "commercial": False,
                "note": "source の権利未確認。公開前に必ず確認すること。"},
}

# 手法シグナル → 既知の failure mode（registry）。artifact のフィールドから検出して引く。
_FAILURE_REGISTRY: dict[str, tuple[str, str]] = {
    "mediapipe": ("perception",
                  "MediaPipe は単眼 2D→近似 3D 推定: 奥行きが曖昧で、オクルージョン・"
                  "自己交差・速い動きで姿勢が乱れる。世界スケール/global trajectory は持たない。"),
    "hmr": ("perception",
            "HMR adapter は skeleton-first（近似 rest offset・betas/shape 未使用）。"
            "形状個体差が rest に反映されず、single-person 前提。"),
    "gvhmr": ("perception", "GVHMR は world-grounded だが本 adapter は betas を使わない近似 FK。"),
    "4dhumans": ("perception", "4DHumans は weak-perspective camera 近似で root 並進が近似。"),
    "smpl_fk": ("perception", "SMPL FK は SMPL body model file を使わない近似 rest offset。"),
    "kinematic": ("retarget",
                  "kinematic retarget は direction-preserving FK のみ: 接触保存/関節 limit 最適化を"
                  "行わず、動的実現可能性は保証しない（sim_certificate で別途検証）。"),
    "actuator_space_ik": ("retarget",
                          "actuator-space IK は参照 IK（位置合わせ）であり、バランス policy ではない。"
                          "torso/toe は合成 target、近似質量。"),
    "differentiable_fk_gradient_ik": ("retarget",
                                      "勾配 IK は局所解依存で、限られた DOF では追従誤差が残る。"),
    "mujoco": ("simulation",
               "sim_certificate は近似質量・慣性（bone 長比）で計算する physically-informed "
               "feasibility であり、実機保証ではない。"),
    "rl_tracking_policy": ("control",
                           "RL tracking policy は v0 baseline: 近似質量・単一/少数参照で、"
                           "PD 超えの精度・摂動頑健性・実機転移は未達。"),
    "policy": ("control",
               "control_mode=policy の軌道は学習方策の物理ロールアウト: 物理的に妥当でも"
               "実機での安定は別途 safety guard / 実機検証が必要。"),
}


def _license_section(state: str) -> dict[str, Any]:
    info = _LICENSE_INFO.get(state, _LICENSE_INFO["unknown"])
    return {"state": state, **info}


def _collect_failures(signals: list[str]) -> list[dict[str, str]]:
    """検出したシグナル列から重複を除いた failure mode 一覧を作る。"""
    seen: set[str] = set()
    out: list[dict[str, str]] = []
    for sig in signals:
        for key, (area, desc) in _FAILURE_REGISTRY.items():
            if key in sig and key not in seen:
                seen.add(key)
                out.append({"area": area, "description": desc})
    return out


def build_mir_card(mir: RdMir) -> dict[str, Any]:
    """RD-MIR の Model Card（dict）を生成する。"""
    ev = mir.extractor_versions or {}
    signals = [str(v) for v in ev.values()]
    signals += [str(mir.source_ref.get("extractor", ""))]
    n = len(mir.keypoints_3d) if mir.keypoints_3d else round(mir.fps * mir.duration)
    lineage = [
        {"stage": "source", "detail": _fmt_source(mir.source_ref)},
        {"stage": "extraction", "detail": _fmt_dict(ev) or "（不明）"},
    ]
    return {
        "rd_card_version": CARD_VERSION,
        "card_type": "mir",
        "identity": {
            "id": mir.motion_id,
            "fps": mir.fps,
            "duration_s": round(mir.duration, 3),
            "frames": n,
            "skeleton_joints": len(mir.skeleton.joint_names),
            "action_label": (mir.semantics or {}).get("action_label", "unknown"),
        },
        "lineage": lineage,
        "license": _license_section(mir.license_state),
        "privacy": mir.privacy_flags or {},
        "intended_use": [
            "ヒューマノイドへの retarget 元 motion（canonical 19-joint RD-MIR）",
            "motion 検索 / 埋め込み / 生成の入力",
        ],
        "out_of_scope": [
            "license_state を満たさない再配布・商用利用",
            "実機コマンドへの直接利用（retarget → sim_certificate → safety guard を経ること）",
        ],
        "failure_modes": _collect_failures(signals),
        "safety_limits": {
            "note": "RD-MIR は kinematic 表現。実機安全は下流（sim_certificate / safety guard）で担保。",
        },
        "metrics": mir.quality_metrics or {},
        "provenance": {"source_ref": mir.source_ref, "extractor_versions": ev},
    }


def build_motion_card(motion: RdMotion, *, mir: Optional[RdMir] = None) -> dict[str, Any]:
    """RD-Motion の Model Card（dict）を生成する。mir を渡すと license/source を継承する。"""
    prov = motion.source_provenance or {}
    metrics = motion.retarget_metrics or {}
    cert = motion.sim_certificate or {}
    signals = [str(prov.get("method", "")), str(metrics.get("method", "")),
               str(motion.control_mode), str(cert.get("backend", ""))]

    lineage: list[dict[str, str]] = []
    if mir is not None:
        lineage.append({"stage": "source_mir", "detail":
                        f"{mir.motion_id}（license={mir.license_state}, "
                        f"extractor={_fmt_dict(mir.extractor_versions or {})}）"})
    else:
        lineage.append({"stage": "source_mir",
                        "detail": f"{motion.source_motion_id}（RD-MIR を別途参照）"})
    lineage.append({"stage": "retarget",
                    "detail": f"{prov.get('method', metrics.get('method', '不明'))} → {motion.robot_name}"})
    if cert:
        lineage.append({"stage": "sim_certificate",
                        "detail": f"{cert.get('backend', '?')}: {cert.get('verdict', '未検証')}"})
    lineage.append({"stage": "control_mode", "detail": motion.control_mode})

    # license は RD-Motion 自体は持たない → source RD-MIR から継承（無ければ unknown）。
    lic_state = mir.license_state if mir is not None else "unknown"

    has_joints = bool(motion.joint_rotations)
    safety: dict[str, Any] = {}
    if cert:
        safety["sim_certificate"] = {
            "verdict": cert.get("verdict"),
            "thresholds": cert.get("thresholds", {}),
            "metrics": cert.get("metrics", {}),
            "reasons": cert.get("reasons", []),
        }
    if has_joints:
        jr = motion.joint_rotations or {}
        safety["actuator"] = {
            "actuated_joints": len(jr.get("actuated_joint_names", [])),
            "note": "実機コマンド直前に joint-space safety guard（位置/速度/加速度クランプ）を通すこと。",
        }

    return {
        "rd_card_version": CARD_VERSION,
        "card_type": "motion",
        "identity": {
            "id": motion.source_motion_id,
            "robot": motion.robot_name,
            "fps": motion.fps,
            "duration_s": round(motion.duration, 3),
            "frames": motion.num_frames,
            "control_mode": motion.control_mode,
            "has_actuator_angles": has_joints,
        },
        "lineage": lineage,
        "license": _license_section(lic_state),
        "intended_use": [
            f"{motion.robot_name} の sim 再生 / 可視化（RViz, viewer）",
            "sim_certificate PASS かつ safety guard 通過時の、慎重な実機評価（tethered, 低速）",
        ],
        "out_of_scope": [
            "sim_certificate 無し / REJECT の motion の実機再生",
            "safety guard を経由しない直接コマンド送出",
            "license_state が許さない再配布・商用利用",
        ],
        "failure_modes": _collect_failures(signals),
        "safety_limits": safety or {"note": "sim_certificate 未計算。validate-sim で検証すること。"},
        "metrics": metrics,
        "provenance": prov,
    }


def license_composition(states: list[str]) -> dict[str, Any]:
    """license_state のリスト → 構成内訳（collection 用 license composition）。"""
    counts: dict[str, int] = {}
    for s in states:
        counts[s] = counts.get(s, 0) + 1
    total = len(states) or 1
    redistributable = sum(c for s, c in counts.items()
                          if _LICENSE_INFO.get(s, {}).get("redistribution"))
    return {
        "total": len(states),
        "by_state": dict(sorted(counts.items(), key=lambda kv: -kv[1])),
        "redistributable_fraction": round(redistributable / total, 3),
    }


def render_markdown(card: dict[str, Any]) -> str:
    """カード dict を Markdown に整形する。"""
    ident = card.get("identity", {})
    title = ident.get("id", "artifact")
    lines = [
        f"# RobotDance {card.get('card_type', '').upper()} Card — `{title}`",
        "",
        f"_rd_card_version {card.get('rd_card_version', CARD_VERSION)} · "
        "⚠️ pre-alpha (v0): 既知の限界を正直に明示する目的のカード_",
        "",
        "## Identity",
    ]
    for k, v in ident.items():
        lines.append(f"- **{k}**: {v}")

    lines += ["", "## Data Lineage"]
    for st in card.get("lineage", []):
        lines.append(f"1. **{st['stage']}** — {st['detail']}")

    lic = card.get("license", {})
    lines += ["", "## License", f"- **state**: `{lic.get('state')}`",
              f"- 再配布可: {lic.get('redistribution')} / 商用可: {lic.get('commercial')}",
              f"- {lic.get('note', '')}"]

    lines += ["", "## Intended Use"]
    lines += [f"- {u}" for u in card.get("intended_use", [])]
    lines += ["", "## Out of Scope"]
    lines += [f"- {u}" for u in card.get("out_of_scope", [])]

    lines += ["", "## Failure Modes / Known Limitations (v0)"]
    fms = card.get("failure_modes", [])
    if fms:
        for fm in fms:
            lines.append(f"- **[{fm['area']}]** {fm['description']}")
    else:
        lines.append("- （検出された既知 failure mode なし — それでも pre-alpha である点に注意）")

    lines += ["", "## Safety Limits"]
    lines.append("```json")
    import json

    lines.append(json.dumps(card.get("safety_limits", {}), ensure_ascii=False, indent=2))
    lines.append("```")

    if card.get("metrics"):
        lines += ["", "## Metrics", "```json",
                  json.dumps(card["metrics"], ensure_ascii=False, indent=2), "```"]

    lines += ["", "---", "_Generated by robotdance model-card (§7). "
              "RobotDance は v0 pre-alpha であり、実機保証ではない。_"]
    return "\n".join(lines) + "\n"


# --- helpers ---

def _fmt_dict(d: dict[str, Any]) -> str:
    return ", ".join(f"{k}={v}" for k, v in d.items())


def _fmt_source(src: dict[str, Any]) -> str:
    parts = []
    for key in ("dataset_name", "extractor", "local_path", "frame"):
        if key in src:
            parts.append(f"{key}={src[key]}")
    return ", ".join(parts) or _fmt_dict(src) or "（不明）"
