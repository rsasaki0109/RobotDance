"""robotdance_unitree

G1/H1 configs, URDF mapping, SDK2/ROS2 bridge assumptions — Unitree を primary target とする embodiment 統合。
"""

from robotdance_retarget.embodiment import RobotMorphology

from . import g1, h1

# robot 名 → 形態 の registry。新しい Unitree 機種はここに追加する。
EMBODIMENTS: dict[str, RobotMorphology] = {
    g1.ROBOT_NAME: g1.MORPHOLOGY,
    h1.ROBOT_NAME: h1.MORPHOLOGY,
}


def get_morphology(name: str) -> RobotMorphology:
    """robot 名から RobotMorphology を返す。"""
    if name not in EMBODIMENTS:
        raise KeyError(f"未知の robot: {name}（利用可能: {sorted(EMBODIMENTS)}）")
    return EMBODIMENTS[name]


__all__ = ["EMBODIMENTS", "get_morphology", "g1", "h1"]
