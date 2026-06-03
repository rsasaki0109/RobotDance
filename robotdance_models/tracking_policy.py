"""RL tracking policy baseline（§4.5, v0）。

`robotdance_sim.tracking_env.TrackingEnv`（MuJoCo 物理上の reference-tracking 環境）で、
**参照運動を倒れずに追従する方策**を小型 PPO で学習する。これは retarget→sim_certificate の
「物理的に妥当か」の判定の次にある「**実際にバランスを取って動かせるか**」の足場であり、
設計書 §4.5（Sim-to-Real Policy Stack）の最初のベースラインに当たる。

方策は base 非駆動の underactuated humanoid に対し関節空間 PD への残差トルクを出力する。
学習後、`TrackingPolicy.rollout()` が物理ロールアウトを RD-Motion（control_mode="policy"）として
返すので、そのまま viewer / sim_certificate / ROS2 の既存パイプラインに流せる。

⚠️ v0: 単一参照・単一環境・小規模 PPO の **baseline**。多様な motion への汎化、AMP/敵対報酬、
領域ランダム化、実機転移は今後。近似質量・素朴な報酬ゆえ tracking は完全ではない。
torch / mujoco が必要（`[learn]` / `[sim]` extra、CI には含めない）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np
import torch
import torch.nn as nn

from robotdance_core.rd_motion import RdMotion, Skeleton
from robotdance_core.skeleton import JOINT_NAMES, PARENTS
from robotdance_retarget.embodiment import RobotMorphology
from robotdance_sim.tracking_env import TrackingEnv


class ActorCritic(nn.Module):
    """対角ガウス方策 + 状態価値の共有 MLP。"""

    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 128) -> None:
        super().__init__()
        self.body = nn.Sequential(
            nn.Linear(obs_dim, hidden), nn.Tanh(),
            nn.Linear(hidden, hidden), nn.Tanh(),
        )
        self.mu = nn.Linear(hidden, act_dim)
        # 小さめの初期探索（PD baseline を壊さない）。mu 初期 0 → 学習前は PD のみ。
        self.log_std = nn.Parameter(torch.full((act_dim,), -1.6))
        self.value = nn.Linear(hidden, 1)
        nn.init.zeros_(self.mu.weight)
        nn.init.zeros_(self.mu.bias)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        h = self.body(x)
        std = self.log_std.exp().expand_as(self.mu(h))
        return self.mu(h), std, self.value(h).squeeze(-1)


def train_tracking_policy(
    reference: RdMotion,
    morphology: RobotMorphology,
    *,
    iterations: int = 40,
    steps_per_iter: int = 1024,
    gamma: float = 0.97,
    lam: float = 0.95,
    clip: float = 0.2,
    lr: float = 3e-4,
    epochs: int = 6,
    hidden: int = 128,
    device: Optional[str] = None,
    seed: int = 0,
    out_path: Optional[str | Path] = None,
) -> tuple["TrackingPolicy", dict[str, Any]]:
    """PPO で tracking 方策を学習し、(TrackingPolicy, info) を返す。"""
    dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(seed)
    np.random.seed(seed)

    env = TrackingEnv(reference, morphology)
    ac = ActorCritic(env.obs_dim, env.action_dim, hidden=hidden).to(dev)
    opt = torch.optim.Adam(ac.parameters(), lr=lr)

    return_hist: list[float] = []
    rmse_hist: list[float] = []
    for _ in range(iterations):
        obs_b, act_b, logp_b, rew_b, val_b, done_b = [], [], [], [], [], []
        ep_returns: list[float] = []
        ep_rmse: list[float] = []
        ep_ret = 0.0
        ep_err: list[float] = []
        o = env.reset()
        for _ in range(steps_per_iter):
            ot = torch.as_tensor(o, device=dev)
            with torch.no_grad():
                mu, std, val = ac(ot)
                dist = torch.distributions.Normal(mu, std)
                a = dist.sample()
                logp = float(dist.log_prob(a).sum())
            o2, r, d, info = env.step(a.cpu().numpy())
            obs_b.append(o)
            act_b.append(a.cpu().numpy())
            logp_b.append(logp)
            rew_b.append(r)
            val_b.append(float(val))
            done_b.append(d)
            ep_ret += r
            ep_err.append(info["pose_rmse"])
            o = o2
            if d:
                ep_returns.append(ep_ret)
                ep_rmse.append(float(np.mean(ep_err)))
                ep_ret = 0.0
                ep_err = []
                o = env.reset()

        with torch.no_grad():
            _, _, last_val = ac(torch.as_tensor(o, device=dev))
        vals = val_b + [float(last_val)]
        adv = np.zeros(len(rew_b), dtype=np.float64)
        gae = 0.0
        for t in reversed(range(len(rew_b))):
            nonterm = 0.0 if done_b[t] else 1.0
            delta = rew_b[t] + gamma * vals[t + 1] * nonterm - vals[t]
            gae = delta + gamma * lam * nonterm * gae
            adv[t] = gae
        ret = adv + np.asarray(val_b)
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        obs_t = torch.as_tensor(np.asarray(obs_b), device=dev)
        act_t = torch.as_tensor(np.asarray(act_b), device=dev, dtype=torch.float32)
        logp_old = torch.as_tensor(np.asarray(logp_b), device=dev, dtype=torch.float32)
        adv_t = torch.as_tensor(adv, device=dev, dtype=torch.float32)
        ret_t = torch.as_tensor(ret, device=dev, dtype=torch.float32)
        for _ in range(epochs):
            mu, std, val = ac(obs_t)
            dist = torch.distributions.Normal(mu, std)
            logp = dist.log_prob(act_t).sum(-1)
            ratio = torch.exp(logp - logp_old)
            l1 = ratio * adv_t
            l2 = torch.clamp(ratio, 1 - clip, 1 + clip) * adv_t
            pi_loss = -torch.min(l1, l2).mean()
            v_loss = ((val - ret_t) ** 2).mean()
            ent = dist.entropy().sum(-1).mean()
            loss = pi_loss + 0.5 * v_loss - 0.01 * ent
            opt.zero_grad()
            loss.backward()
            opt.step()

        return_hist.append(float(np.mean(ep_returns)) if ep_returns else ep_ret)
        rmse_hist.append(float(np.mean(ep_rmse)) if ep_rmse else float("nan"))

    policy = TrackingPolicy(ac, env, dev)
    info = {
        "device": dev,
        "obs_dim": env.obs_dim,
        "action_dim": env.action_dim,
        "actuated_dofs": env.n_act,
        "ref_frames": env.T,
        "iterations": iterations,
        "return_history": return_hist,
        "rmse_history": rmse_hist,
    }
    if out_path is not None:
        out_path = Path(out_path)
        torch.save(
            {
                "state_dict": ac.state_dict(),
                "obs_dim": env.obs_dim,
                "action_dim": env.action_dim,
                "hidden": hidden,
            },
            out_path,
        )
        info["checkpoint"] = str(out_path)
    return policy, info


class TrackingPolicy:
    """学習済み tracking 方策。決定論的ロールアウトを RD-Motion として返す。"""

    def __init__(self, ac: ActorCritic, env: TrackingEnv, device: str) -> None:
        self.ac = ac
        self.env = env
        self.device = device

    @torch.no_grad()
    def rollout(self) -> tuple[RdMotion, dict[str, Any]]:
        """方策（平均行動）で物理ロールアウトし、追従結果を RD-Motion で返す。"""
        env = self.env
        o = env.reset()
        kps = [env.current_keypoints()]
        errs: list[float] = []
        survived = 0
        for _ in range(env.T - 1):
            mu, _std, _v = self.ac(torch.as_tensor(o, device=self.device))
            o, _r, d, info = env.step(mu.cpu().numpy())
            kps.append(env.current_keypoints())
            errs.append(info["pose_rmse"])
            survived += 1
            if d:
                break

        arr = np.stack(kps)  # [F, J, 3]
        n_frames = arr.shape[0]
        metrics = {
            "method": "rl_tracking_policy_ppo",
            "survived_frames": survived,
            "reference_frames": env.T - 1,
            "survival_ratio": round(survived / max(env.T - 1, 1), 3),
            "mean_pose_rmse": round(float(np.mean(errs)) if errs else float("nan"), 4),
            "note": (
                "PPO tracking baseline（v0）。base 非駆動の物理ロールアウト。"
                "近似質量・単一参照ゆえ完全追従ではない。retarget→sim_certificate とは別物。"
            ),
        }
        motion = RdMotion(
            robot_name=env.morph.name,
            fps=env.fps,
            duration=n_frames / env.fps,
            source_motion_id=env.reference.source_motion_id,
            skeleton=Skeleton(joint_names=list(JOINT_NAMES), parents=list(PARENTS)),
            control_mode="policy",
            keypoints_3d=arr.tolist(),
            contact_schedule=env.reference.contact_schedule,
            retarget_metrics=metrics,
            source_provenance={
                "rd_motion_source_id": env.reference.source_motion_id,
                "method": "rl_tracking_policy",
                "robot": env.morph.name,
            },
        )
        return motion, metrics
