#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    """
    Merge joint states from:
      - UR arms on /ur_joint_states (generic names)
      - Left Allegro on /la_/joint_states (renaming to la_joint_...)
      - Right Allegro on /ra_/joint_states (renaming to ra_joint_...)
    """

    def __init__(self):
        super().__init__('joint_state_merger')

        # ---- Parameters ----
        self.declare_parameter('ur_topic', '/ur_joint_states')
        self.declare_parameter('la_allegro_topic', '/la_/joint_states')
        self.declare_parameter('ra_allegro_topic', '/ra_/joint_states')
        self.declare_parameter('publish_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 100.0)

        ur_topic = self.get_parameter('ur_topic').get_parameter_value().string_value
        la_topic = self.get_parameter('la_allegro_topic').get_parameter_value().string_value
        ra_topic = self.get_parameter('ra_allegro_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        rate_hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value or 100.0)
        period_s = 1.0 / max(rate_hz, 1.0)

        qos = QoSProfile(depth=10)

        # ---- Internal State ----
        self.ur_state = None
        self.la_state = None
        self.ra_state = None

        # ---- Subscriptions ----
        self.sub_ur = self.create_subscription(JointState, ur_topic, self._cb_ur, qos)
        self.sub_la = self.create_subscription(JointState, la_topic, self._cb_la, qos)
        self.sub_ra = self.create_subscription(JointState, ra_topic, self._cb_ra, qos)

        # ---- Publisher ----
        self.pub_merged = self.create_publisher(JointState, pub_topic, qos)

        # ---- Timer ----
        self.timer = self.create_timer(period_s, self._publish_merged)

        self.get_logger().info(f"Merger initialized. Publishing to {pub_topic}")

    def _cb_ur(self, msg: JointState):
        self.ur_state = msg

    def _cb_la(self, msg: JointState):
        # Rename joints from "joint_0_0" to "la_joint_0_0"
        msg.name = [f"la_{name}" if not name.startswith("la_") else name for name in msg.name]
        self.la_state = msg

    def _cb_ra(self, msg: JointState):
        # Rename joints from "joint_0_0" to "ra_joint_0_0"
        msg.name = [f"ra_{name}" if not name.startswith("ra_") else name for name in msg.name]
        self.ra_state = msg

    def _publish_merged(self):
        # Only publish if we have data from all components to avoid "ghosting" in RViz
        if self.ur_state is None or self.la_state is None or self.ra_state is None:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        # Combine names and positions
        merged.name = list(self.ur_state.name) + list(self.la_state.name) + list(self.ra_state.name)
        merged.position = list(self.ur_state.position) + list(self.la_state.position) + list(self.ra_state.position)

        # Handle Velocity
        if self._all_have('velocity'):
            merged.velocity = list(self.ur_state.velocity) + list(self.la_state.velocity) + list(self.ra_state.velocity)
        else:
            merged.velocity = []

        # Handle Effort
        if self._all_have('effort'):
            merged.effort = list(self.ur_state.effort) + list(self.la_state.effort) + list(self.ra_state.effort)
        else:
            merged.effort = []

        self.pub_merged.publish(merged)

    def _all_have(self, field: str) -> bool:
        """Utility to check if data exists across all topics for a field."""
        try:
            a = getattr(self.ur_state, field, [])
            b = getattr(self.la_state, field, [])
            c = getattr(self.ra_state, field, [])
            return len(a) > 0 and len(b) > 0 and len(c) > 0
        except:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()