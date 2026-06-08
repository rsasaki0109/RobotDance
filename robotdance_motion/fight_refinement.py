"""実動画由来 motion を fight / assisted 向けに深度精緻化するパイプライン。

`extract --stabilize-depth`（抽出側）と `validate-sim --balance-refine`（retarget 側）を
HumanoidBattle の前処理として束ねる。観測 y/z は凍結し、未観測の前後 x だけを扱う。
"""

from __future__ import annotations

from robotdance_core.rd_mir import RdMir


def refine_for_fight(
    mir: RdMir,
    *,
    stabilize: bool = True,
    balance: bool = True,
    balance_strength: float = 0.4,
) -> RdMir:
    """fight / assisted 前の RD-MIR 深度フロンティア（quasi-static 前提）。"""
    m = mir
    if stabilize:
        from robotdance_motion.depth_stabilize import stabilize_depth

        m = stabilize_depth(m)
    if balance:
        from robotdance_motion.depth_refine import balance_depth_refine

        m = balance_depth_refine(m, strength=balance_strength)
    qm = dict(m.quality_metrics or {})
    qm["fight_refinement"] = {
        "stabilize": stabilize,
        "balance": balance,
        "balance_strength": balance_strength if balance else None,
    }
    return m.model_copy(update={"quality_metrics": qm})


__all__ = ["refine_for_fight"]
