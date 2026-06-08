"""Unified retarget dispatch — builtin kinematic vs external GMR."""

from __future__ import annotations

from robotdance_core.rd_mir import RdMir
from robotdance_core.rd_motion import RdMotion
from robotdance_retarget.backends import get_retarget_backend
from robotdance_retarget.embodiment import RobotMorphology


def retarget_with_backend(
    mir: RdMir,
    morphology: RobotMorphology,
    backend: str = "kinematic",
    *,
    clamp_flexion: bool = False,
    conf_gate: float | None = None,
    gmr_verbose: bool = False,
) -> RdMotion:
    """Retarget RD-MIR with the named backend."""
    b = get_retarget_backend(backend)
    if b.name == "kinematic":
        from robotdance_retarget.kinematic import retarget

        return retarget(
            mir, morphology, clamp_flexion=clamp_flexion, conf_gate=conf_gate,
        )
    if b.name == "gmr":
        from robotdance_retarget.gmr_backend import gmr_retarget

        if clamp_flexion or conf_gate is not None:
            raise ValueError("GMR backend は --clamp-flexion / --conf-gate 未対応（v0.153）")
        return gmr_retarget(mir, morphology, verbose=gmr_verbose)
    raise ValueError(f"retarget backend '{backend}' は実行未配線です")


__all__ = ["retarget_with_backend"]
