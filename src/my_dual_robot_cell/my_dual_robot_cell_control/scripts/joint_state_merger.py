#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    """
    Merge joint states from:
      - UR arms (both) on /ur_joint_states
      - Left Allegro on /la_/allegroHand_1/joint_states
      - Right Allegro on /ra_/allegroHand_0/joint_states
    and publish a single /joint_states for the whole dual-arm+hands system.
    """

    def __init__(self):
        super().__init__('joint_state_merger')

        # ---- Parameters (so you can adjust topics without code changes) ----
        self.declare_parameter('ur_topic', '/ur_joint_states')
        self.declare_parameter('la_allegro_topic', '/la_/allegroHand_1/joint_states')
        self.declare_parameter('ra_allegro_topic', '/ra_/allegroHand_0/joint_states')
        self.declare_parameter('publish_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 100.0)  # 100 Hz by default

        ur_topic = self.get_parameter('ur_topic').get_parameter_value().string_value
        la_topic = self.get_parameter('la_allegro_topic').get_parameter_value().string_value
        ra_topic = self.get_parameter('ra_allegro_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        rate_hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value or 100.0)
        period_s = 1.0 / max(rate_hz, 1.0)

        qos = QoSProfile(depth=50)

        # ---- Subscriptions ----
        self.ur_state = None
        self.la_state = None
        self.ra_state = None

        self.sub_ur = self.create_subscription(JointState, ur_topic, self._cb_ur, qos)
        self.sub_la = self.create_subscription(JointState, la_topic, self._cb_la, qos)
        self.sub_ra = self.create_subscription(JointState, ra_topic, self._cb_ra, qos)

        # ---- Publisher ----
        self.pub_merged = self.create_publisher(JointState, pub_topic, qos)

        # ---- Timer ----
        self.timer = self.create_timer(period_s, self._publish_merged)

        self.get_logger().info(
            f"JointStateMerger up:\n"
            f"  UR topic:  {ur_topic}\n"
            f"  LA topic:  {la_topic}\n"
            f"  RA topic:  {ra_topic}\n"
            f"  Publish:   {pub_topic} @ {rate_hz:.1f} Hz"
        )

    # ---- Callbacks ----
    def _cb_ur(self, msg: JointState):
        self.ur_state = msg

    def _cb_la(self, msg: JointState):
        self.la_state = msg

    def _cb_ra(self, msg: JointState):
        self.ra_state = msg

    # ---- Merge & Publish ----
    def _publish_merged(self):
        # Wait until we have at least one message from all three sources
        if self.ur_state is None or self.la_state is None or self.ra_state is None:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        # Names / positions always required
        merged.name = list(self.ur_state.name) + list(self.la_state.name) + list(self.ra_state.name)
        merged.position = list(self.ur_state.position) + list(self.la_state.position) + list(self.ra_state.position)

        # Only publish velocity if all three provide them (non-empty)
        if self._all_have('velocity'):
            merged.velocity = list(self.ur_state.velocity) + list(self.la_state.velocity) + list(self.ra_state.velocity)
        else:
            merged.velocity = []

        # Only publish effort if all three provide them (non-empty)
        if self._all_have('effort'):
            merged.effort = list(self.ur_state.effort) + list(self.la_state.effort) + list(self.ra_state.effort)
        else:
            merged.effort = []

        self.pub_merged.publish(merged)

    def _all_have(self, field: str) -> bool:
        """Return True if ur/la/ra states all have a non-empty list for the given field."""
        a = getattr(self.ur_state, field, None)
        b = getattr(self.la_state, field, None)
        c = getattr(self.ra_state, field, None)
        try:
            return (a is not None and len(a) > 0) and (b is not None and len(b) > 0) and (c is not None and len(c) > 0)
        except TypeError:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
