"""HumanoidBattle ランキング — ELO・対戦履歴・Hall of Champions を永続化する。

物理 fight トーナメント（`fight_tournament`）の結果を JSON state に蓄積し、
Markdown leaderboard を再生成する。physics benchmark の `LEADERBOARD.md` とは別ファイル。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path
from typing import Any

from robotdance_benchmarks.fight_tournament import FightMatchResult, FightTournamentResult

DEFAULT_STATE_PATH = Path("docs/benchmark/humanoid_battle_state.json")
DEFAULT_MD_PATH = Path("docs/benchmark/HUMANOID_BATTLE_LEADERBOARD.md")


def state_path_for(md_path: Path | None = None) -> Path:
    """Markdown leaderboard と同ディレクトリの JSON state パス。"""
    md = md_path or DEFAULT_MD_PATH
    return md.parent / "humanoid_battle_state.json"

INITIAL_ELO = 1500.0
K_FACTOR = 32.0


def expected_score(rating_a: float, rating_b: float) -> float:
    return 1.0 / (1.0 + 10.0 ** ((rating_b - rating_a) / 400.0))


def update_elo(rating_a: float, rating_b: float, score_a: float) -> tuple[float, float]:
    """score_a: 1.0=勝ち, 0.5=引分, 0.0=負け。"""
    ea = expected_score(rating_a, rating_b)
    eb = expected_score(rating_b, rating_a)
    return rating_a + K_FACTOR * (score_a - ea), rating_b + K_FACTOR * ((1.0 - score_a) - eb)


def _score_for_winner(winner: str, p1: str, p2: str) -> float:
    if winner == "DRAW" or winner == "TIE":
        return 0.5
    if winner == p1:
        return 1.0
    if winner == p2:
        return 0.0
    raise ValueError(f"winner '{winner}' が対戦者 {p1}/{p2} と一致しません")


@dataclass
class BattleLeaderboardState:
    """永続 state（JSON シリアライズ）。"""

    elo: dict[str, float] = field(default_factory=dict)
    hall: list[dict[str, Any]] = field(default_factory=list)
    bouts: list[dict[str, Any]] = field(default_factory=list)
    version: int = 1

    def rating(self, robot: str) -> float:
        return float(self.elo.get(robot, INITIAL_ELO))

    def apply_match(self, match: FightMatchResult, *, styles: list[str], event: str) -> None:
        """1 試合の ELO 更新 + bout ログ追記。"""
        p1, p2 = match.p1, match.p2
        for r in (p1, p2):
            self.elo.setdefault(r, INITIAL_ELO)
        sa = _score_for_winner(match.winner, p1, p2)
        ra, rb = update_elo(self.rating(p1), self.rating(p2), sa)
        self.elo[p1], self.elo[p2] = round(ra, 1), round(rb, 1)
        rs = " ".join(f"{r.style}:{r.p1_hits}-{r.p2_hits}" for r in match.rounds)
        self.bouts.append({
            "date": date.today().isoformat(),
            "event": event,
            "p1": p1,
            "p2": p2,
            "styles": list(styles),
            "rounds": rs,
            "p1_hits": match.p1_total_hits,
            "p2_hits": match.p2_total_hits,
            "winner": match.winner,
            "elo_p1": self.elo[p1],
            "elo_p2": self.elo[p2],
        })

    def crown(self, champion: str, *, mode: str, detail: str, finalist: str) -> None:
        self.hall.append({
            "date": date.today().isoformat(),
            "champion": champion,
            "finalist": finalist,
            "mode": mode,
            "detail": detail,
        })


def load_state(path: Path | None = None) -> BattleLeaderboardState:
    path = path or DEFAULT_STATE_PATH
    if not path.is_file():
        return BattleLeaderboardState()
    data = json.loads(path.read_text(encoding="utf-8"))
    return BattleLeaderboardState(
        elo={k: float(v) for k, v in (data.get("elo") or {}).items()},
        hall=list(data.get("hall") or []),
        bouts=list(data.get("bouts") or []),
        version=int(data.get("version", 1)),
    )


def save_state(state: BattleLeaderboardState, path: Path | None = None) -> Path:
    path = path or DEFAULT_STATE_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": state.version,
        "elo": state.elo,
        "hall": state.hall,
        "bouts": state.bouts,
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def record_fight_tournament(
    result: FightTournamentResult,
    styles: list[str],
    *,
    state_path: Path | None = None,
) -> BattleLeaderboardState:
    """ブラケット全試合を ELO に反映し、チャンピオンを Hall に追加して state を保存。"""
    state = load_state(state_path)
    for rnd in result.bracket:
        for m in rnd:
            state.apply_match(m, styles=styles, event="physical_tournament")
    f = result.final
    finalist = f.p2 if result.champion == f.p1 else f.p1
    detail = f"styles={','.join(styles)} final={f.p1_total_hits}-{f.p2_total_hits} hits"
    state.crown(result.champion, mode="physical_fight", detail=detail, finalist=finalist)
    save_state(state, state_path)
    return state


def record_kinematic_champion(
    champion: str,
    moves: list[str],
    finalist: str,
    *,
    state_path: Path | None = None,
) -> BattleLeaderboardState:
    """型採点トーナメントのチャンピオンのみ Hall に記録（ELO は更新しない）。"""
    state = load_state(state_path)
    state.crown(champion, mode="kata_scoring", detail=f"moves={','.join(moves)}", finalist=finalist)
    save_state(state, state_path)
    return state


def render_markdown(state: BattleLeaderboardState) -> str:
    """state から Markdown leaderboard 本文を生成。"""
    lines = [
        "# HumanoidBattle Leaderboard",
        "",
        "Physical fight トーナメント（`demo-tournament --physical --record`）の ELO・対戦履歴・"
        "チャンピオン記録。physics benchmark とは別（[`LEADERBOARD.md`](LEADERBOARD.md)）。",
        "",
        f"> 最終更新: {date.today().isoformat()} · state: `humanoid_battle_state.json`",
        "",
        "## ELO Rankings",
        "",
        "| rank | robot | ELO | bouts |",
        "| --- | --- | ---: | ---: |",
    ]
    if not state.elo:
        lines.append("| — | *(no bouts yet)* | — | — |")
    else:
        bout_ct: dict[str, int] = {}
        for b in state.bouts:
            bout_ct[b["p1"]] = bout_ct.get(b["p1"], 0) + 1
            bout_ct[b["p2"]] = bout_ct.get(b["p2"], 0) + 1
        ranked = sorted(state.elo.items(), key=lambda x: (-x[1], x[0]))
        for i, (robot, elo) in enumerate(ranked, 1):
            lines.append(f"| {i} | {robot} | {elo:.1f} | {bout_ct.get(robot, 0)} |")

    lines += ["", "## Hall of Champions", "", "| date | mode | champion | finalist | note |", "| --- | --- | --- | --- | --- |"]
    if not state.hall:
        lines.append("| — | — | — | — | — |")
    else:
        for h in reversed(state.hall[-20:]):
            lines.append(
                f"| {h['date']} | {h['mode']} | **{h['champion']}** | {h.get('finalist', '—')} "
                f"| {h.get('detail', '')} |"
            )

    lines += [
        "",
        "## Recent Bouts",
        "",
        "| date | p1 | p2 | rounds | total hits | winner | ELO (p1/p2) |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    if not state.bouts:
        lines.append("| — | — | — | — | — | — | — |")
    else:
        for b in reversed(state.bouts[-30:]):
            hits = f"{b['p1_hits']}-{b['p2_hits']}"
            elo = f"{b.get('elo_p1', '—')}/{b.get('elo_p2', '—')}"
            lines.append(
                f"| {b['date']} | {b['p1']} | {b['p2']} | {b.get('rounds', '')} "
                f"| {hits} | {b['winner']} | {elo} |"
            )
    lines.append("")
    return "\n".join(lines)


def write_leaderboard(
    state: BattleLeaderboardState,
    md_path: Path | None = None,
) -> Path:
    md_path = md_path or DEFAULT_MD_PATH
    md_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text(render_markdown(state), encoding="utf-8")
    return md_path


def record_and_write_fight_tournament(
    result: FightTournamentResult,
    styles: list[str],
    *,
    state_path: Path | None = None,
    md_path: Path | None = None,
) -> Path:
    md = md_path or DEFAULT_MD_PATH
    sp = state_path or state_path_for(md)
    state = record_fight_tournament(result, styles, state_path=sp)
    return write_leaderboard(state, md)


__all__ = [
    "BattleLeaderboardState",
    "DEFAULT_MD_PATH",
    "DEFAULT_STATE_PATH",
    "state_path_for",
    "INITIAL_ELO",
    "K_FACTOR",
    "expected_score",
    "load_state",
    "record_and_write_fight_tournament",
    "record_fight_tournament",
    "record_kinematic_champion",
    "render_markdown",
    "save_state",
    "update_elo",
    "write_leaderboard",
]
