"""Headless smoke test of Booster Deploy t2_walk in the NUSim T2 scene.

python test/t2_walk_closed_loop.py --booster-deploy /path/to/booster_deploy
This test drives MuJoCo directly; the ROS/DDS bridge has a separate contract.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--booster-deploy", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--vx", type=float, default=0.3)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vyaw", type=float, default=0.0)
    args = parser.parse_args()
    sys.path.insert(0, str(args.booster_deploy.resolve()))

    import mujoco
    import numpy as np
    import torch
    from booster_deploy.controllers.base_controller import BaseController
    from tasks.locomotion.robots.t2 import T2WalkTaskCfg

    cfg = T2WalkTaskCfg()
    cfg.policy.device = "cpu"
    model_path = Path(__file__).resolve().parents[1] / "models/t2/t2_scene_robocup_middle.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready"))
    names = cfg.robot.joint_names
    qadr = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names])
    vadr = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names])
    aids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in names])
    if min(aids) < 0:
        raise RuntimeError("T2 actuator mapping incomplete")

    class Controller(BaseController):
        def ctrl_step(self, targets):
            pass

        def update_state(self):
            state = self.robot.data
            state.joint_pos = torch.tensor(data.qpos[qadr], dtype=torch.float32)
            state.joint_vel = torch.tensor(data.qvel[vadr], dtype=torch.float32)
            state.feedback_torque = torch.tensor(data.qfrc_actuator[vadr], dtype=torch.float32)
            state.root_pos_w = torch.tensor(data.qpos[:3], dtype=torch.float32)
            state.root_quat_w = torch.tensor(data.qpos[3:7], dtype=torch.float32)
            state.root_lin_vel_b = torch.tensor(data.qvel[:3], dtype=torch.float32)
            state.root_ang_vel_b = torch.tensor(data.qvel[3:6], dtype=torch.float32)

        def run(self):
            pass

    controller = Controller(cfg)
    controller.vel_command.lin_vel_x = args.vx
    controller.vel_command.lin_vel_y = args.vy
    controller.vel_command.ang_vel_yaw = args.vyaw
    controller.update_state()
    controller.start()
    kp = np.asarray(cfg.robot.joint_stiffness)
    kd = np.asarray(cfg.robot.joint_damping)
    limit = np.asarray(cfg.robot.effort_limit)
    steps_per_policy = round(cfg.policy_dt / model.opt.timestep)
    z_min = float(data.qpos[2])
    for _ in range(round(args.seconds / cfg.policy_dt)):
        controller.update_state()
        targets = controller.policy_step().detach().cpu().numpy()
        for _ in range(steps_per_policy):
            tau = np.clip(kp * (targets - data.qpos[qadr]) - kd * data.qvel[vadr], -limit, limit)
            data.ctrl[aids] = tau
            mujoco.mj_step(model, data)
            z_min = min(z_min, float(data.qpos[2]))
    w, x, y, z = data.qpos[3:7]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    print(f"x={data.qpos[0]:.3f} y={data.qpos[1]:.3f} z={data.qpos[2]:.3f} yaw={yaw:.3f} min_z={z_min:.3f}")
    if not np.isfinite(data.qpos).all() or z_min < 0.5:
        raise SystemExit("T2 policy failed closed-loop smoke test")


if __name__ == "__main__":
    main()
