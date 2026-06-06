"""RD-MIR の健全性チェック（単眼抽出のよくある破綻を自動検出）。

実動画→ヒューマノイドのデバッグで繰り返し当たった失敗モードを、再利用可能な診断にまとめる:

- **mirror**（左右反転）: 背面撮影だと MediaPipe が前向き前提で左右を取り違える。canonical は y:左
  なので、立位で left_hip.y > right_hip.y が正常。符号が逆なら反転を疑う（squat rear-view で発生）。
- **depth_collapse**（深度なし）: 前後 x の分散が極端に小さい。2D→planar lift では設計どおりだが、
  native 3D で起きていれば抽出不良。
- **foot_skate / not_grounded**（接地不良）: 足の最下点が z=0 から離れる、または接地足が横滑り。
- **multi_subject**（多人数）: quality_metrics["n_subjects_max"] > 1。前景以外を掴むと overlay がズレる。
- **low_confidence / jitter**: 平均 confidence が低い、平滑後 jitter が大きい。

各チェックは ok/warn/info の status と、人が読める message・対処ヒントを返す純関数。
関連: [[real-video-demo-pipeline]] [[pose-backend-registry]]。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from robotdance_core.skeleton import index_of

if TYPE_CHECKING:
    from robotdance_core.rd_mir import RdMir


@dataclass
class Check:
    """1 つの診断結果。status は "ok" | "warn" | "info"。"""

    name: str
    status: str
    message: str
    hint: str = ""


def _mirror_check(kps: np.ndarray) -> Check:
    """canonical y:左 → 立位で left_hip.y > right_hip.y が正常。逆なら左右反転を疑う。"""
    dy = float((kps[:, index_of("left_hip"), 1] - kps[:, index_of("right_hip"), 1]).mean())
    if dy < 0:
        return Check("mirror", "warn",
                     f"左右反転の疑い（mean(left_hip.y - right_hip.y)={dy:+.3f} < 0）",
                     "背面/真後ろからの撮影が原因のことが多い。正面・側面のクリップを使う。")
    return Check("mirror", "ok", f"左右の向きは正常（hip 幅 y={dy:+.3f}）")


def _depth_check(kps: np.ndarray, *, planar_expected: bool) -> Check:
    """前後 x の分散が縦 z に比して極端に小さければ深度崩壊。lift では設計どおり。"""
    sx = float(kps[:, :, 0].std())
    sz = float(kps[:, :, 2].std())
    ratio = sx / sz if sz > 1e-9 else 0.0
    if ratio < 0.05:
        if planar_expected:
            return Check("depth_collapse", "info",
                         f"深度ほぼゼロ（x/z std={ratio:.3f}）。planar lift では設計どおり。")
        return Check("depth_collapse", "warn",
                     f"深度（前後 x）がほぼ無い（x/z std={ratio:.3f}）",
                     "矢状面の動きが潰れる。正面/側面寄りのクリップか深度復元が必要。")
    return Check("depth_collapse", "ok", f"深度の変動あり（x/z std={ratio:.3f}）")


def _grounding_check(kps: np.ndarray) -> Check:
    """足の最下点が z=0 付近にあるか、各フレームの最下点の散らばりで接地を見る。"""
    feet = [index_of("left_foot"), index_of("right_foot"),
            index_of("left_ankle"), index_of("right_ankle")]
    per_frame_low = kps[:, feet, 2].min(axis=1)
    spread = float(per_frame_low.max() - per_frame_low.min())
    if spread > 0.15:
        return Check("grounding", "warn",
                     f"接地高さが安定しない（最下点の振れ {spread:.3f} m）",
                     "単眼の根高さ誤差/foot skate。validate-sim --ground-clean を試す。")
    return Check("grounding", "ok", f"接地は概ね安定（最下点の振れ {spread:.3f} m）")


def _confidence_check(mir: "RdMir") -> Check:
    q = mir.quality_metrics or {}
    mc = q.get("mean_confidence")
    if mc is None:
        return Check("confidence", "info", "mean_confidence の記録なし")
    if mc < 0.6:
        return Check("confidence", "warn", f"平均 confidence が低い（{mc}）",
                     "被写体が小さい/遮蔽/暗い可能性。クリップを見直す。")
    return Check("confidence", "ok", f"平均 confidence {mc}")


def _jitter_check(mir: "RdMir") -> Check:
    q = mir.quality_metrics or {}
    j = q.get("jitter_after", q.get("jitter_before"))
    if j is None:
        return Check("jitter", "info", "jitter の記録なし")
    if j > 0.05:
        return Check("jitter", "warn", f"平滑後も jitter が大きい（{j}）",
                     "検出が不安定。smoothing 強化や別検出器を検討。")
    return Check("jitter", "ok", f"jitter {j}")


def _subject_check(mir: "RdMir") -> Check:
    q = mir.quality_metrics or {}
    n = q.get("n_subjects_max")
    if n is None:
        return Check("multi_subject", "info", "検出人数の記録なし")
    if n > 1:
        return Check("multi_subject", "warn", f"複数人を検出（最大 {n} 人）",
                     "前景の主被写体に固定済みだが、意図と違えば演武者領域に crop する。")
    return Check("multi_subject", "ok", "単一被写体")


def diagnose_motion(mir: "RdMir") -> list[Check]:
    """RD-MIR を診断し Check のリストを返す。keypoints_3d 必須。"""
    kps = mir.keypoints_3d_array()
    q = mir.quality_metrics or {}
    planar_expected = bool(q.get("lift"))  # planar lift 由来なら深度ゼロは設計どおり
    return [
        _mirror_check(kps),
        _depth_check(kps, planar_expected=planar_expected),
        _grounding_check(kps),
        _confidence_check(mir),
        _jitter_check(mir),
        _subject_check(mir),
    ]


def overall_status(checks: list[Check]) -> str:
    """全 Check から総合 status（warn が 1 つでもあれば "warn"）。"""
    return "warn" if any(c.status == "warn" for c in checks) else "ok"


def warn_names(checks: list[Check]) -> list[str]:
    """warn 状態の Check 名のリスト（コーパス集計用）。"""
    return [c.name for c in checks if c.status == "warn"]
