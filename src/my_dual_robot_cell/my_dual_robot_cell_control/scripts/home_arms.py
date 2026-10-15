#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from allegro_hand_interfaces.srv import SetDesPos
from rclpy.parameter import Parameter

class HomeArmsNode(Node):
    def __init__(self):
        super().__init__('home_arms_node')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        self.la_pub = self.create_publisher(Float64MultiArray, '/la_forward_position_controller/commands', 10)
        self.ra_pub = self.create_publisher(Float64MultiArray, '/ra_forward_position_controller/commands', 10)
        
        self.lh_client = self.create_client(SetDesPos, '/la_/setDesiredConfiguration')
        self.rh_client = self.create_client(SetDesPos, '/ra_/setDesiredConfiguration')
        
        # State persistence (Crucial for split packets)
        self.current_la = None
        self.current_ra = None
        
        self.la_order = ["la_shoulder_pan_joint", "la_shoulder_lift_joint", "la_elbow_joint", "la_wrist_1_joint", "la_wrist_2_joint", "la_wrist_3_joint"]
        self.ra_order = ["ra_shoulder_pan_joint", "ra_shoulder_lift_joint", "ra_elbow_joint", "ra_wrist_1_joint", "ra_wrist_2_joint", "ra_wrist_3_joint"]
        
        self.create_subscription(JointState, '/ur_joint_states', self._joint_cb, 10)
        
        self.la_home_pos = [math.radians(-18.75), math.radians(-138.64), math.radians(-99.59), math.radians(57.97), math.radians(64.88), math.radians(137.67)]
        self.ra_home_pos = [math.radians(27.33), math.radians(-43.98), math.radians(98.18), math.radians(-232.88), math.radians(-71.98), math.radians(223.84)]
        self.hand_open_pos = [0.0, -0.15, -0.15, -0.15, 0.0, -0.15, -0.15, -0.15, 0.0, -0.15, -0.15, -0.15, 0.50, -0.10, -0.10, -0.10]

    def _joint_cb(self, msg):
        m = dict(zip(msg.name, msg.position))
        # Persistently update - do not overwrite with None
        lq = [m.get(jn) for jn in self.la_order]
        if all(v is not None for v in lq): self.current_la = lq
        
        rq = [m.get(jn) for jn in self.ra_order]
        if all(v is not None for v in rq): self.current_ra = rq

    def open_hands(self):
        req = SetDesPos.Request(pos=self.hand_open_pos)
        if self.lh_client.wait_for_service(timeout_sec=1.0): self.lh_client.call_async(req)
        if self.rh_client.wait_for_service(timeout_sec=1.0): self.rh_client.call_async(req)

    def move_to_home(self):
        self.get_logger().info('Waiting for hardware feedback (Checking both arms)...')
        while rclpy.ok():
            if self.current_la is not None and self.current_ra is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Hardware detected. Moving to home position...')
        start_t, dur = time.time(), 5.0
        q0_la, q0_ra = list(self.current_la), list(self.current_ra)

        while rclpy.ok():
            elapsed = time.time() - start_t
            s = min(1.0, elapsed / dur)
            smooth_s = (3.0 * s**2) - (2.0 * s**3)

            self.la_pub.publish(Float64MultiArray(data=[a + smooth_s*(b-a) for a,b in zip(q0_la, self.la_home_pos)]))
            self.ra_pub.publish(Float64MultiArray(data=[a + smooth_s*(b-a) for a,b in zip(q0_ra, self.ra_home_pos)]))
            rclpy.spin_once(self, timeout_sec=0.0)  # flush outgoing messages

            if s >= 1.0:
                break
            time.sleep(0.01)  # 100 Hz — use wall-clock sleep, not create_rate()

def main():
    rclpy.init()
    node = HomeArmsNode()
    node.open_hands()
    time.sleep(0.5)
    node.move_to_home()
    node.get_logger().info('Home sequence complete.')
    # Let the final publish flush before tearing down
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
