#!/usr/bin/env python3
"""
Unified Dual-Arm Controller - Closed-Loop Anti-Swing Version
Uses physical feedback to damp commands and prevent oscillation/circular swinging.
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.parameter import Parameter

class DualArmController(Node):
    def __init__(self):
        super().__init__("dual_arm_controller")
        
        # ========== PARAMETERS ==========
        self.publish_rate = 100.0
        self.EMA_BETA = 0.45       # 55% new / 45% old — responsive to policy commands (was 0.85)
        self.V_MAX = 1.5           # rad/s — matches policy training dynamics (was 0.6)
        self.A_MAX = 1.2           # rad/s² — soft acceleration ramp
        self.K_P = 2.5             # Proportional gain: rad/s per rad of position error
        self.K_D = 3.0             # Damping gain: rad/s per rad of hardware lag (q_real - q_cmd)
        # Stop publishing when no new targets arrive for this many seconds.
        # Prevents fighting with home_arms.py after the policy ends.
        self.TARGET_TIMEOUT_SEC = 1.0
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        # ========== CONFIG ==========
        self.arms = {
            'left': {
                'order': ["la_shoulder_pan_joint", "la_shoulder_lift_joint", "la_elbow_joint", "la_wrist_1_joint", "la_wrist_2_joint", "la_wrist_3_joint"],
                'topic': "/la_forward_position_controller/commands",
                'target': None, 'robot_q': None, 'cmd_q': None, 'cmd_v': [0.0]*6
            },
            'right': {
                'order': ["ra_shoulder_pan_joint", "ra_shoulder_lift_joint", "ra_elbow_joint", "ra_wrist_1_joint", "ra_wrist_2_joint", "ra_wrist_3_joint"],
                'topic': "/ra_forward_position_controller/commands",
                'target': None, 'robot_q': None, 'cmd_q': None, 'cmd_v': [0.0]*6
            }
        }
        
        self.pubs = {n: self.create_publisher(Float64MultiArray, c['topic'], 1) for n,c in self.arms.items()}
        self.create_subscription(JointState, "/joint_states_re_left", lambda m: self._on_sim(m, 'left'), 10)
        self.create_subscription(JointState, "/joint_states_re_right", lambda m: self._on_sim(m, 'right'), 10)
        self.create_subscription(JointState, "/ur_joint_states", self._on_hw, 10)

        # Track when we last received a command for each arm (for timeout logic).
        self._last_cmd_time = {'left': 0.0, 'right': 0.0}

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info("Dual-Arm Closed-Loop Controller Active")

    def _on_hw(self, msg: JointState):
        m = dict(zip(msg.name, msg.position))
        for name, arm in self.arms.items():
            q = [m.get(jn) for jn in arm['order']]
            if all(v is not None for v in q): arm['robot_q'] = q

    def _on_sim(self, msg: JointState, arm_name: str):
        arm = self.arms[arm_name]
        try:
            q_in = [msg.position[msg.name.index(jn)] for jn in arm['order']]
        except (ValueError, IndexError): return

        self._last_cmd_time[arm_name] = time.time()

        if arm['target'] is None: arm['target'] = q_in
        else:
            b = self.EMA_BETA
            arm['target'] = [b * p + (1.0 - b) * c for p, c in zip(arm['target'], q_in)]

    def _tick(self):
        dt = 1.0 / self.publish_rate
        now = time.time()
        for name, arm in self.arms.items():
            # Clear stale target so we stop publishing after the policy ends.
            # This prevents fighting with home_arms.py when it takes over.
            if arm['target'] is not None and (now - self._last_cmd_time[name]) > self.TARGET_TIMEOUT_SEC:
                self.get_logger().info(f"[{name}] No commands for {self.TARGET_TIMEOUT_SEC:.1f}s — releasing arm.", once=True)
                arm['target'] = None
                arm['cmd_q']  = None
                arm['cmd_v']  = [0.0] * 6
            if arm['target'] is None or arm['robot_q'] is None: continue
            
            # Start from hardware
            if arm['cmd_q'] is None:
                arm['cmd_q'] = list(arm['robot_q'])
                continue

            q_goal = arm['target']
            q_curr = arm['cmd_q']
            v_curr = arm['cmd_v']
            q_real = arm['robot_q']

            new_q, new_v = [], []
            for i in range(6):
                # PD velocity law — no /dt division, so errors never get 100x amplified.
                # K_P drives q_cmd toward q_goal; K_D brakes when hardware lags behind q_cmd.
                v_req = (self.K_P * (q_goal[i] - q_curr[i])
                         + self.K_D * (q_real[i] - q_curr[i]))

                v_clamped = max(-self.V_MAX, min(self.V_MAX, v_req))
                
                dv = v_clamped - v_curr[i]
                dv_max = self.A_MAX * dt
                dv_clamped = max(-dv_max, min(dv_max, dv))
                
                v_final = v_curr[i] + dv_clamped
                new_q.append(q_curr[i] + v_final * dt)
                new_v.append(v_final)

            self.pubs[name].publish(Float64MultiArray(data=new_q))
            arm['cmd_q'], arm['cmd_v'] = new_q, new_v

def main():
    rclpy.init()
    try: rclpy.spin(DualArmController())
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
