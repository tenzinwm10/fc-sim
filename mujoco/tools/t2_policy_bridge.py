"""Run Booster Deploy's t2_walk against NUSim's T2 DDS surface.

Source ROS 2 plus booster_interface/booster_msgs and put the pinned
booster_deploy checkout on PYTHONPATH before running this script. NUSim owns
the sole RPC response writer. This process only observes Move requests and
publishes the official policy's 31 joint targets to /joint_ctrl.
"""

from __future__ import annotations

import argparse
import json
import time


def main() -> None:
    import rclpy
    import torch
    from booster_interface.msg import LowCmd, LowState, MotorCmd
    from booster_msgs.msg import RpcReqMsg
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    from booster_deploy.controllers.base_controller import BaseController
    from booster_deploy.utils.isaaclab import math as lab_math
    from tasks.locomotion.robots.t2 import T2WalkTaskCfg

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--command-timeout", type=float, default=0.3)
    args = parser.parse_args()

    cfg = T2WalkTaskCfg()
    cfg.policy.device = args.device

    class PolicyController(BaseController):
        def ctrl_step(self, targets):
            pass  # ROS publisher below owns the output.

        def update_state(self):
            pass  # Latest LowState callback updates the robot data.

        def run(self):
            pass  # ROS timer drives policy steps.

    controller = PolicyController(cfg)

    class Bridge(Node):
        def __init__(self):
            super().__init__("fcsim_t2_policy")
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.low_pub = self.create_publisher(LowCmd, "joint_ctrl", qos)
            self.create_subscription(LowState, "low_state", self.on_state, qos)
            self.create_subscription(RpcReqMsg, "LocoApiTopicReq", self.on_rpc, qos)
            self.state_time = 0.0
            self.command_time = 0.0
            self.started = False
            self.create_timer(cfg.policy_dt, self.tick)

        def on_state(self, msg):
            if len(msg.motor_state_serial) != controller.robot.num_joints:
                self.get_logger().error(
                    f"LowState has {len(msg.motor_state_serial)} joints; expected 31"
                )
                return
            data = controller.robot.data
            device = data.device
            data.joint_pos = torch.tensor(
                [m.q for m in msg.motor_state_serial], dtype=torch.float32, device=device
            )
            data.joint_vel = torch.tensor(
                [m.dq for m in msg.motor_state_serial], dtype=torch.float32, device=device
            )
            data.feedback_torque = torch.tensor(
                [m.tau_est for m in msg.motor_state_serial], dtype=torch.float32, device=device
            )
            rpy = torch.tensor(msg.imu_state.rpy, dtype=torch.float32, device=device)
            data.root_quat_w = lab_math.quat_from_euler_xyz(*rpy).reshape(4)
            data.root_ang_vel_b = torch.tensor(
                msg.imu_state.gyro, dtype=torch.float32, device=device
            )
            self.state_time = time.monotonic()

        def on_rpc(self, msg):
            try:
                if int(json.loads(msg.header).get("api_id", -1)) != 2001:
                    return
                body = json.loads(msg.body)
                command = controller.vel_command
                command.lin_vel_x = command.clamp_vx(float(body["vx"]))
                command.lin_vel_y = max(-command.vy_max, min(command.vy_max, float(body["vy"])))
                command.ang_vel_yaw = max(
                    -command.vyaw_max, min(command.vyaw_max, float(body["vyaw"]))
                )
                self.command_time = time.monotonic()
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                self.get_logger().warning(f"Invalid Move RPC: {exc}")

        def tick(self):
            now = time.monotonic()
            if now - self.state_time > 0.2:
                return
            if not self.started:
                controller.start()
                self.started = True
                self.get_logger().info("Booster Deploy t2_walk policy started")
            if not controller.is_running:
                return  # safety fallback stopped inference; let the stream expire
            if now - self.command_time > args.command_timeout:
                command = controller.vel_command
                command.lin_vel_x = command.lin_vel_y = command.ang_vel_yaw = 0.0
            targets = controller.policy_step()
            if not controller.is_running:
                return
            cmd = LowCmd()
            cmd.cmd_type = LowCmd.CMD_TYPE_SERIAL
            for i, target in enumerate(targets):
                motor = MotorCmd()
                motor.q = float(target)
                motor.kp = float(controller.robot.joint_stiffness[i])
                motor.kd = float(controller.robot.joint_damping[i])
                motor.dq = 0.0
                motor.tau = 0.0
                cmd.motor_cmd.append(motor)
            self.low_pub.publish(cmd)

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
