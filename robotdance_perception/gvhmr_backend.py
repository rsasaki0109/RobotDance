"""GVHMR in-process video extraction — world-grounded SMPL → RD-MIR.

GVHMR (ZJU, SIGGRAPH Asia 2024) recovers gravity-view / world-grounded human motion from
monocular video. RobotDance runs GVHMR inference in-process when the cloned repo is installed
with checkpoints, then converts ``smpl_params_global`` to canonical RD-MIR via ``from_gvhmr``.

Requires a full GVHMR install (not PyPI). Clone + deps + checkpoint download:

    git clone https://github.com/zju3dv/GVHMR.git
    cd GVHMR && pip install -e .   # follow docs/INSTALL.md for torch/CUDA deps
    # download inputs/checkpoints/gvhmr/gvhmr_siga24_release.ckpt per GVHMR README

Optional: ``ROBOTDANCE_GVHMR_ROOT`` points at the clone if ``hmr4d`` is on PYTHONPATH
without the repo root (for relative assets).

GPU (CUDA) is required for practical inference — GVHMR's demo pipeline moves batches to CUDA.
"""

from __future__ import annotations

import importlib.util
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from robotdance_core.rd_mir import RdMir
from robotdance_perception.hmr import from_gvhmr

_DEFAULT_CKPT = Path("inputs/checkpoints/gvhmr/gvhmr_siga24_release.ckpt")


def gvhmr_importable() -> bool:
    return importlib.util.find_spec("hmr4d") is not None


def _gvhmr_root() -> Path | None:
    env = os.environ.get("ROBOTDANCE_GVHMR_ROOT")
    if env:
        root = Path(env).expanduser().resolve()
        if (root / "hmr4d").is_dir():
            return root
    if not gvhmr_importable():
        return None
    try:
        import hmr4d

        root = Path(hmr4d.__file__).resolve().parent.parent
        if (root / "hmr4d").is_dir():
            return root
    except Exception:
        return None
    return None


def gvhmr_checkpoint_available() -> bool:
    root = _gvhmr_root()
    if root is None:
        return False
    return (root / _DEFAULT_CKPT).is_file()


def gvhmr_cuda_available() -> bool:
    try:
        import torch

        return torch.cuda.is_available()
    except Exception:
        return False


def gvhmr_available() -> bool:
    return gvhmr_importable() and gvhmr_checkpoint_available() and gvhmr_cuda_available()


def gvhmr_install_hint() -> str:
    return (
        "GVHMR が未導入、checkpoint 不足、または CUDA がありません。clone + 依存 + ckpt が必要:\n"
        "  git clone https://github.com/zju3dv/GVHMR.git\n"
        "  cd GVHMR && pip install -e .   # docs/INSTALL.md 参照（torch/CUDA）\n"
        "  # inputs/checkpoints/gvhmr/gvhmr_siga24_release.ckpt を配置\n"
        "  export ROBOTDANCE_GVHMR_ROOT=/path/to/GVHMR  # 任意"
    )


@contextmanager
def _gvhmr_workdir(root: Path):
    prev = os.getcwd()
    os.chdir(root)
    try:
        yield
    finally:
        os.chdir(prev)


def _tensor_dict_to_numpy(params: dict[str, Any]) -> dict[str, Any]:
    import numpy as np
    import torch

    out: dict[str, Any] = {}
    for key, val in params.items():
        if torch.is_tensor(val):
            out[key] = val.detach().cpu().numpy()
        else:
            out[key] = np.asarray(val)
    return out


def _pred_to_gvhmr_result(pred: dict[str, Any]) -> dict[str, Any]:
    return {
        "smpl_params_global": _tensor_dict_to_numpy(pred["smpl_params_global"]),
        "smpl_params_incam": _tensor_dict_to_numpy(pred["smpl_params_incam"]),
    }


def _run_gvhmr_inference(
    video_path: Path,
    *,
    static_cam: bool,
    use_dpvo: bool,
    f_mm: int | None,
    verbose: bool,
    output_root: Path | None,
) -> dict[str, Any]:
    """Run GVHMR preprocess + predict; return native pred dict (torch tensors)."""
    import torch
    from hydra import compose, initialize_config_module
    from pytorch3d.transforms import quaternion_to_matrix

    from hmr4d.configs import register_store_gvhmr
    from hmr4d.model.gvhmr.gvhmr_pl_demo import DemoPL
    from hmr4d.utils.geo.hmr_cam import convert_K_to_K4, create_camera_sensor, estimate_K
    from hmr4d.utils.geo_transform import compute_cam_angvel
    from hmr4d.utils.preproc import Extractor, SimpleVO, Tracker, VitPoseExtractor
    from hmr4d.utils.pylogger import Log
    from hmr4d.utils.video_io_utils import get_video_lwh

    root = _gvhmr_root()
    if root is None:
        raise RuntimeError(gvhmr_install_hint())

    video_path = video_path.resolve()
    if not video_path.is_file():
        raise FileNotFoundError(f"動画が見つかりません: {video_path}")

    out_root = output_root or Path(tempfile.mkdtemp(prefix="robotdance_gvhmr_"))
    length, width, height = get_video_lwh(video_path)

    with initialize_config_module(version_base="1.3", config_module="hmr4d.configs"):
        register_store_gvhmr()
        overrides = [
            f"video_name={video_path.stem}",
            f"static_cam={static_cam}",
            f"verbose={verbose}",
            f"use_dpvo={use_dpvo}",
            f"output_root={out_root}",
            f"video_path={video_path}",
        ]
        if f_mm is not None:
            overrides.append(f"f_mm={f_mm}")
        cfg = compose(config_name="demo", overrides=overrides)

    paths = cfg.paths
    Path(cfg.preprocess_dir).mkdir(parents=True, exist_ok=True)
    Path(cfg.output_dir).mkdir(parents=True, exist_ok=True)

    with _gvhmr_workdir(root):
        # --- preprocess (cache under output_dir/preprocess) ---
        if not Path(paths.bbx).exists():
            from hmr4d.utils.geo.hmr_cam import get_bbx_xys_from_xyxy

            tracker = Tracker()
            bbx_xyxy = tracker.get_one_track(str(video_path)).float()
            bbx_xys = get_bbx_xys_from_xyxy(bbx_xyxy, base_enlarge=1.2).float()
            torch.save({"bbx_xyxy": bbx_xyxy, "bbx_xys": bbx_xys}, paths.bbx)
            del tracker
        bbx_xys = torch.load(paths.bbx)["bbx_xys"]

        if not Path(paths.vitpose).exists():
            vitpose_extractor = VitPoseExtractor()
            vitpose = vitpose_extractor.extract(str(video_path), bbx_xys)
            torch.save(vitpose, paths.vitpose)
            del vitpose_extractor
        vitpose = torch.load(paths.vitpose)

        if not Path(paths.vit_features).exists():
            extractor = Extractor()
            vit_features = extractor.extract_video_features(str(video_path), bbx_xys)
            torch.save(vit_features, paths.vit_features)
            del extractor

        if not static_cam and not Path(paths.slam).exists():
            if use_dpvo:
                from hmr4d.utils.preproc.slam import SLAMModel

                K_fullimg = estimate_K(width, height)
                intrinsics = convert_K_to_K4(K_fullimg)
                slam = SLAMModel(str(video_path), width, height, intrinsics, buffer=4000, resize=0.5)
                while slam.track():
                    pass
                traj = slam.process()
            else:
                simple_vo = SimpleVO(str(video_path), scale=0.5, step=8, method="sift", f_mm=cfg.f_mm)
                traj = simple_vo.compute()
            torch.save(traj, paths.slam)

        if static_cam:
            R_w2c = torch.eye(3).repeat(length, 1, 1)
        else:
            traj = torch.load(paths.slam)
            if use_dpvo:
                traj_quat = torch.from_numpy(traj[:, [6, 3, 4, 5]])
                R_w2c = quaternion_to_matrix(traj_quat).mT
            else:
                R_w2c = torch.from_numpy(traj[:, :3, :3])

        if f_mm is not None:
            K_fullimg = create_camera_sensor(width, height, f_mm)[2].repeat(length, 1, 1)
        else:
            K_fullimg = estimate_K(width, height).repeat(length, 1, 1)

        data = {
            "length": torch.tensor(length),
            "bbx_xys": torch.load(paths.bbx)["bbx_xys"],
            "kp2d": vitpose,
            "K_fullimg": K_fullimg,
            "cam_angvel": compute_cam_angvel(R_w2c),
            "f_imgseq": torch.load(paths.vit_features),
        }

        if Path(paths.hmr4d_results).exists():
            pred = torch.load(paths.hmr4d_results)
            Log.info(f"[GVHMR] cached results from {paths.hmr4d_results}")
            return pred

        import hydra

        Log.info("[GVHMR] predicting SMPL (world-grounded)")
        model: DemoPL = hydra.utils.instantiate(cfg.model, _recursive_=False)
        ckpt = root / _DEFAULT_CKPT
        model.load_pretrained_model(str(ckpt))
        model = model.eval().cuda()
        pred = model.predict(data, static_cam=static_cam)
        torch.save(pred, paths.hmr4d_results)
        return pred


def extract_gvhmr_video(
    video_path: str | Path,
    *,
    static_cam: bool = True,
    use_dpvo: bool = False,
    f_mm: int | None = None,
    verbose: bool = False,
    fps: float = 30.0,
    output_root: Path | None = None,
) -> RdMir:
    """Run GVHMR on *video_path* and return canonical RD-MIR."""
    if not gvhmr_available():
        raise RuntimeError(gvhmr_install_hint())

    video_path = Path(video_path)
    pred = _run_gvhmr_inference(
        video_path, static_cam=static_cam, use_dpvo=use_dpvo,
        f_mm=f_mm, verbose=verbose, output_root=output_root,
    )
    result = _pred_to_gvhmr_result(pred)
    mir = from_gvhmr(result, world=True, fps=fps)
    mir.source_ref = {
        **(mir.source_ref or {}),
        "local_path": str(video_path.resolve()),
        "extractor": "gvhmr_inprocess",
        "static_cam": static_cam,
    }
    q = dict(mir.quality_metrics or {})
    q["world_grounded"] = True
    q["gvhmr_static_cam"] = static_cam
    mir.quality_metrics = q
    return mir


__all__ = [
    "extract_gvhmr_video",
    "gvhmr_available",
    "gvhmr_checkpoint_available",
    "gvhmr_cuda_available",
    "gvhmr_importable",
    "gvhmr_install_hint",
]
