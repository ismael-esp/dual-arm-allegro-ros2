#!/usr/bin/env python3
"""
Real-time Allegro Hand Service Commander (Right Hand)

Subscribes to joint angles from Isaac Lab simulation and commands
the real right Allegro hand via the SetDesiredConfiguration service.

Features:
- Subscribes to /joint_states_right_hand from Isaac Lab
- Applies delay buffering & interpolation for smooth motion
- Calls /ra_/setDesiredConfiguration service in real-time loop
- No flickering or erratic motion
"""

import time
from typing import List, Optional
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from allegro_hand_interfaces.srv import SetDesPos


class AllegroHandServiceCommanderRight(Node):
    def __init__(self):
        super().__init__('allegro_hand_service_commander_right')

        # ====== Parameters ======
        # Delay to synchronize with arm (seconds)
        self.delay_s = float(
            self.declare_parameter('delay_s', 0.00).get_parameter_value().double_value #0.5
        )
        
        # Command publish rate (Hz)
        self.publish_rate = float(
            self.declare_parameter('publish_rate', 100.0).get_parameter_value().double_value
        )
        
        # Buffer maximum storage time (seconds)
        self.buffer_secs = float(
            self.declare_parameter('buffer_secs', 0.5).get_parameter_value().double_value
        )

        # Release the hand when the input stream goes silent (e.g. bodex / Isaac
        # Lab stopped publishing) so EXTERNAL setDesiredConfiguration / setController
        # calls take effect again WITHOUT a driver restart. Must exceed any real
        # stream's inter-message gap (a 100 Hz stream is 0.01 s).
        self.stale_timeout = float(
            self.declare_parameter('stale_timeout', 0.4).get_parameter_value().double_value
        )
        self._last_rx = None   # wall-clock (s) of the last received hand message

        # ====== Service Client ======
        self.service_client = self.create_client(
            SetDesPos, '/ra_/setDesiredConfiguration'
        )
        
        # Wait for service to be available
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for service /ra_/setDesiredConfiguration... '
                'Ensure Allegro hand driver is running (slave:=True)'
            )

        # ====== Subscription ======
        self.sub = self.create_subscription(
            JointState, '/joint_states_right_hand', self.callback, 100
        )

        # Output order (Right Allegro hand - 16 DOF)
        self.output_order = [
            "ra_joint_0_0", "ra_joint_1_0", "ra_joint_2_0", "ra_joint_3_0",
            "ra_joint_4_0", "ra_joint_5_0", "ra_joint_6_0", "ra_joint_7_0",
            "ra_joint_8_0", "ra_joint_9_0", "ra_joint_10_0", "ra_joint_11_0",
            "ra_joint_12_0", "ra_joint_13_0", "ra_joint_14_0", "ra_joint_15_0",
        ]

        # ====== Buffer Setup ======
        # Buffer stores: (timestamp_sec, positions[list], velocities[list], efforts[list])
        maxlen = int(self.buffer_secs * max(50.0, self.publish_rate))
        self.buf = deque(maxlen=maxlen)

        # ====== Command Timer ======
        # Timer for real-time service calls
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self._tick)

        self.get_logger().info(
            f"[AllegroHandServiceCommanderRight] Initialized\n"
            f"  Input: /joint_states_right_hand\n"
            f"  Service: /ra_/setDesiredConfiguration\n"
            f"  Delay: {self.delay_s}s, Rate: {self.publish_rate}Hz, "
            f"Buffer: {self.buffer_secs}s"
        )

    @staticmethod
    def _to_sec(stamp) -> float:
        """Convert ROS timestamp to seconds"""
        sec = float(getattr(stamp, 'sec', 0))
        nsec = float(getattr(stamp, 'nanosec', 0))
        t = sec + nsec * 1e-9
        return t

    def callback(self, msg_in: JointState):
        """
        Receive joint states from Isaac Lab and buffer them.
        Apply delay and interpolation in _tick().
        """
        # Reorganize input to match output order
        data = {name: i for i, name in enumerate(msg_in.name)}
        pos, vel, eff = [], [], []
        
        for j in self.output_order:
            if j in data:
                idx = data[j]
                pos.append(msg_in.position[idx] if len(msg_in.position) > idx else 0.0)
                vel.append(msg_in.velocity[idx] if len(msg_in.velocity) > idx else 0.0)
                eff.append(msg_in.effort[idx] if len(msg_in.effort) > idx else 0.0)
            else:
                pos.append(0.0)
                vel.append(0.0)
                eff.append(0.0)

        # Get timestamp
        t_in = self._to_sec(msg_in.header.stamp)
        if t_in <= 0.0:
            # Fallback to local time if timestamp invalid
            t_in = self.get_clock().now().nanoseconds * 1e-9

        # Mark receipt time (wall clock) for the staleness release in _tick().
        self._last_rx = self.get_clock().now().nanoseconds * 1e-9
        # Buffer this sample
        self.buf.append((t_in, pos, vel, eff))

    def _tick(self):
        """
        Periodic callback: 
        1. Extract delayed joint state from buffer (with interpolation)
        2. Call service to command the Allegro hand
        """
        if not self.buf:
            return

        # Input stream silent for > stale_timeout -> RELEASE: stop re-commanding
        # the last pose so an external setDesiredConfiguration/setController wins.
        now_rx = self.get_clock().now().nanoseconds * 1e-9
        if self._last_rx is None or (now_rx - self._last_rx) > self.stale_timeout:
            self.buf.clear()
            return

        # Calculate target time (delay_s in the past)
        now = self.get_clock().now().nanoseconds * 1e-9
        target_t = now - self.delay_s

        # Find buffer samples surrounding target_t
        while len(self.buf) >= 2 and self.buf[1][0] <= target_t:
            self.buf.popleft()

        # Get first sample
        t0, p0, v0, e0 = self.buf[0]
        
        # Get second sample if available (for interpolation)
        if len(self.buf) >= 2:
            t1, p1, v1, e1 = self.buf[1]
        else:
            t1, p1, v1, e1 = t0, p0, v0, e0

        # Linear interpolation between samples
        if t1 > t0:
            alpha = (target_t - t0) / (t1 - t0)
            alpha = max(0.0, min(1.0, alpha))  # Clamp [0, 1]
            pos = [p0[i] + alpha * (p1[i] - p0[i]) for i in range(len(p0))]
            vel = [v0[i] + alpha * (v1[i] - v0[i]) for i in range(len(v0))]
            eff = [e0[i] + alpha * (e1[i] - e0[i]) for i in range(len(e0))]
        else:
            pos, vel, eff = p0, v0, e0

        # Call service to command hand
        self._send_hand_command(pos)

    def _send_hand_command(self, positions: List[float]):
        """
        Call the SetDesiredConfiguration service with interpolated joint positions.
        
        Args:
            positions: List of 16 joint angles (radians) for Allegro hand
        """
        if len(positions) != 16:
            self.get_logger().warn(
                f"Invalid hand joint count: {len(positions)} (expected 16)"
            )
            return

        # Create service request
        request = SetDesPos.Request()
        request.pos = positions

        # Call service asynchronously (non-blocking)
        self.service_client.call_async(request)

    def _send_hand_command_sync(self, positions: List[float]):
        """
        Synchronous service call (blocks until response).
        Use this if you need confirmation before proceeding.
        
        Args:
            positions: List of 16 joint angles (radians) for Allegro hand
        """
        if len(positions) != 16:
            self.get_logger().warn(
                f"Invalid hand joint count: {len(positions)} (expected 16)"
            )
            return

        request = SetDesPos.Request()
        request.pos = positions
        
        try:
            # This blocks until response received
            future = self.service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            if future.done():
                response = future.result()
                return response
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = AllegroHandServiceCommanderRight()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
