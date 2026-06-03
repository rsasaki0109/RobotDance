"""Extraction benchmark ハーネス（§4.1, v0）。

video→RD-MIR の抽出 adapter（MediaPipe / HMR(4DHumans/GVHMR) 等）を、**共通の ground-truth
RD-MIR に対して定量比較**する。入口の品質を可視化し、どの adapter をいつ使うかの判断材料にする。

指標（skeleton 空間, 画像不要）:
  - **MPJPE**（root-relative, m）: 平均関節位置誤差。
  - **PA-MPJPE**（Procrustes 整列後, m）: 相似変換（回転+並進+スケール）で合わせた後の誤差。
  - **PCK@5cm / @10cm**: しきい内に収まった関節割合。
  - **MPJVE**（m/s）: 関節速度誤差（動きの忠実度）。
  - **jitter**（accel ノルム）: 時間的滑らかさ（小さいほど安定）。
  - **bone-length MAE**（m）: 骨長の一貫性（skeleton 安定性）。

⚠️ v0: 評価 *ハーネス* と指標を提供する。実 adapter 比較は実 video の抽出結果（と GT）を渡して
行う。同梱デモは合成 GT に **MediaPipe 風（奥行きノイズ+jitter）/ HMR 風（骨長近似+安定）** の
劣化を加えて harness を実演するもので、実モデルの精度主張ではない。
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np

from robotdance_core.rd_mir import RdMir
from robotdance_core.skeleton import BONES, index_of


def _umeyama_align(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """src [N,3] を dst [N,3] に相似変換（回転+並進+スケール, Umeyama 1991）で整列して返す。"""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = (dc.T @ sc) / src.shape[0]
    u, s, vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(u @ vt))
    diag = np.diag([1.0, 1.0, d])
    r = u @ diag @ vt
    var_s = (sc ** 2).sum() / src.shape[0]
    scale = float(np.trace(np.diag(s) @ diag) / var_s) if var_s > 1e-12 else 1.0
    return (scale * (r @ sc.T).T) + mu_d


def extraction_metrics(
    gt_kp: np.ndarray, pred_kp: np.ndarray, *, fps: float, pck_thresholds=(0.05, 0.10)
) -> dict[str, Any]:
    """ground-truth [T,J,3] と予測 [T,J,3] から抽出品質指標を計算する。"""
    t = min(gt_kp.shape[0], pred_kp.shape[0])
    gt = np.asarray(gt_kp[:t], dtype=np.float64)
    pr = np.asarray(pred_kp[:t], dtype=np.float64)
    pelvis = index_of("pelvis")

    # root-relative（位置・向きの絶対オフセットを除いた純粋な姿勢誤差）。
    gt_r = gt - gt[:, pelvis:pelvis + 1, :]
    pr_r = pr - pr[:, pelvis:pelvis + 1, :]
    per_joint = np.linalg.norm(pr_r - gt_r, axis=2)  # [T,J]
    mpjpe = float(per_joint.mean())

    # PA-MPJPE: フレームごとに相似整列。
    pa_errs = []
    for f in range(t):
        aligned = _umeyama_align(pr[f], gt[f])
        pa_errs.append(np.linalg.norm(aligned - gt[f], axis=1).mean())
    pa_mpjpe = float(np.mean(pa_errs))

    pck = {f"pck@{int(th * 100)}cm": round(float((per_joint <= th).mean()), 3)
           for th in pck_thresholds}

    # 速度誤差（root-relative）。
    if t > 1:
        gv = np.diff(gt_r, axis=0) * fps
        pv = np.diff(pr_r, axis=0) * fps
        mpjve = float(np.linalg.norm(pv - gv, axis=2).mean())
    else:
        mpjve = 0.0

    # jitter（accel ノルム平均）。
    def _jit(a: np.ndarray) -> float:
        return float(np.linalg.norm(np.diff(a, n=2, axis=0), axis=2).mean()) if t > 2 else 0.0

    # bone-length MAE（時間平均骨長の差）。
    def _bonelen(a: np.ndarray) -> np.ndarray:
        return np.array([np.linalg.norm(a[:, j] - a[:, p], axis=1).mean() for j, p in BONES])

    bone_mae = float(np.abs(_bonelen(pr) - _bonelen(gt)).mean())

    return {
        "frames": t,
        "mpjpe_m": round(mpjpe, 4),
        "pa_mpjpe_m": round(pa_mpjpe, 4),
        **pck,
        "mpjve_m_s": round(mpjve, 4),
        "jitter_pred": round(_jit(pr), 5),
        "jitter_gt": round(_jit(gt), 5),
        "bone_length_mae_m": round(bone_mae, 4),
    }


def compare_extractions(
    gt: RdMir, preds: dict[str, RdMir]
) -> list[dict[str, Any]]:
    """GT に対して複数の抽出結果を比較し、1 行 = 1 extractor の指標リストを返す。"""
    gt_kp = gt.keypoints_3d_array()
    rows = []
    for name, mir in preds.items():
        m = extraction_metrics(gt_kp, mir.keypoints_3d_array(), fps=gt.fps)
        rows.append({"extractor": name, **m})
    # MPJPE 昇順（良い順）。
    rows.sort(key=lambda r: r["mpjpe_m"])
    return rows


def synthetic_extraction_demo(*, seed: int = 0) -> tuple[RdMir, dict[str, RdMir]]:
    """合成 GT + シミュレートした extractor 出力（MediaPipe 風 / HMR 風）を作る（harness 実演用）。"""
    from robotdance_core.synthetic import generate_dance

    rng = np.random.default_rng(seed)
    gt = generate_dance(duration=2.0, beats_per_second=1.0)
    gt.motion_id = "gt"
    gt.semantics = {"action_label": "dance"}
    kp = gt.keypoints_3d_array()
    t, j, _ = kp.shape

    # MediaPipe 風: 奥行き(x=forward)に大きめノイズ + フレーム独立 jitter + 軽い等方ノイズ。
    mp = kp.copy()
    mp[:, :, 0] += rng.normal(0, 0.05, size=(t, j))        # 奥行き曖昧性
    mp += rng.normal(0, 0.015, size=(t, j, 3))             # 時間非相関 jitter
    mp_mir = _clone_mir(gt, mp, "mediapipe_like")

    # HMR 風: 時間的に滑らか（相関ノイズ）だが骨長が系統的に近似（skeleton-first）→ scale 1.04。
    smooth = np.cumsum(rng.normal(0, 0.004, size=(t, j, 3)), axis=0)
    smooth -= smooth.mean(axis=0, keepdims=True)
    hmr = kp * 1.04 + smooth                                # 骨長 4% 過大 + 滑らかなドリフト
    hmr_mir = _clone_mir(gt, hmr, "hmr_like")

    return gt, {"mediapipe_like": mp_mir, "hmr_like": hmr_mir}


def _clone_mir(base: RdMir, kp: np.ndarray, motion_id: str) -> RdMir:
    m = base.model_copy(deep=True)
    m.motion_id = motion_id
    m.keypoints_3d = kp.tolist()
    return m


# --- 出力 ---

_COLUMNS = ["extractor", "frames", "mpjpe_m", "pa_mpjpe_m", "pck@5cm", "pck@10cm",
            "mpjve_m_s", "jitter_pred", "jitter_gt", "bone_length_mae_m"]


def write_extraction_csv(rows: list[dict[str, Any]], path: str | Path) -> Path:
    path = Path(path)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in _COLUMNS})
    return path


def render_extraction_markdown(
    rows: list[dict[str, Any]], *, gt_id: str = "gt", title: str = "RobotDance Extraction Benchmark"
) -> str:
    lines = [
        f"# {title}",
        "",
        f"ground-truth: `{gt_id}` · 指標は root-relative（MPJPE/PCK）と相似整列後（PA-MPJPE）。"
        "小さいほど良い（PCK は大きいほど良い）。",
        "",
        "| extractor | MPJPE(m) | PA-MPJPE(m) | PCK@5cm | PCK@10cm | MPJVE(m/s) | jitter | bone-len MAE(m) |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in rows:
        lines.append(
            f"| {r['extractor']} | {r['mpjpe_m']:.4f} | {r['pa_mpjpe_m']:.4f} | "
            f"{r['pck@5cm']:.3f} | {r['pck@10cm']:.3f} | {r['mpjve_m_s']:.4f} | "
            f"{r['jitter_pred']:.5f} | {r['bone_length_mae_m']:.4f} |"
        )
    lines += ["", "_⚠️ v0: 評価ハーネス。同梱デモは合成 GT への模擬劣化であり実モデルの精度主張ではない。_"]
    return "\n".join(lines) + "\n"
