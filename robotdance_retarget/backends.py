"""retarget バックエンドのレジストリ（v0）。

RobotDance の retarget は2つの builtin 経路がある — 速い **kinematic**（関節角への直接マップ）と、
実 URDF のアクチュエータ関節角へ解く **actuator-ik**（実機向け既定）。さらに外部 OSS の
**GMR (General Motion Retargeting, MIT)** が 18 機種・CPU 実時間で動く。本モジュールは各
retarget バックエンドの能力メタデータ（手法・実 URDF 要否・対応入力・導入状況）を 1 か所に束ね、
pose レジストリ（[[pose-backend-registry]] / robotdance_perception.backends）と同じ作法で
一覧・選択できるようにする。

GMR は重い依存（mink/MuJoCo/各 URDF）を要するため RobotDance の依存には含めず、`external` 印で
登録する（available() は遅延 spec チェック、未導入なら一覧で `—`）。関連: docs/RELATED_WORK.md。
"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass, field


@dataclass(frozen=True)
class RetargetBackend:
    """1 つの retarget バックエンドの能力メタデータ。"""

    name: str
    method: str  # "kinematic" / "actuator-ik" / "external"
    real_urdf: bool  # 実 URDF（質量/慣性/可動域）を使うか
    description: str = ""
    modules: tuple[str, ...] = field(default_factory=tuple)  # 必要 import（遅延チェック）
    extras: tuple[str, ...] = field(default_factory=tuple)  # ("external",) など
    cli: str = ""  # builtin の対応 CLI（"retarget" / "retarget-ik"）。external は空。
    url: str = ""  # external の参照 URL

    def available(self) -> bool:
        """必要モジュールが import 可能か（builtin は modules 空で常に True）。"""
        return all(importlib.util.find_spec(m) is not None for m in self.modules)


# builtin: 速い kinematic マップ（CLI `retarget`）。
KINEMATIC = RetargetBackend(
    name="kinematic",
    method="kinematic",
    real_urdf=False,
    description="canonical → ロボット関節角への直接 kinematic マップ。速い。CLI `retarget`。",
    cli="retarget",
)
# builtin: 実 URDF のアクチュエータ関節角へ IK（CLI `retarget-ik`）。実機向け既定。
ACTUATOR_IK = RetargetBackend(
    name="actuator-ik",
    method="actuator-ik",
    real_urdf=True,
    description="実 URDF のアクチュエータ関節角へ IK で解く（実機向け既定）。CLI `retarget-ik`。",
    modules=("pybullet",),
    cli="retarget-ik",
)
# external: GMR（MIT, 18 機種, CPU 実時間, mink+MuJoCo IK）。RobotDance の依存には含めない。
GMR = RetargetBackend(
    name="gmr",
    method="external",
    real_urdf=True,
    description="GMR: General Motion Retargeting (MIT, 18 機種, CPU 実時間 IK)。外部 OSS。",
    modules=("general_motion_retargeting",),
    extras=("external",),
    url="https://github.com/YanjieZe/GMR",
)

_REGISTRY: dict[str, RetargetBackend] = {b.name: b for b in (KINEMATIC, ACTUATOR_IK, GMR)}


def list_retarget_backends() -> list[RetargetBackend]:
    """登録済み retarget バックエンドを名前順で返す。"""
    return [_REGISTRY[k] for k in sorted(_REGISTRY)]


def get_retarget_backend(name: str) -> RetargetBackend:
    """名前から取得。未知なら候補を添えて ValueError。"""
    try:
        return _REGISTRY[name]
    except KeyError:
        known = ", ".join(sorted(_REGISTRY))
        raise ValueError(f"未知の retarget backend '{name}'。利用可能: {known}") from None
