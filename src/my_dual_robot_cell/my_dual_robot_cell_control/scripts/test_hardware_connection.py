#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from allegro_hand_interfaces.srv import SetDesPos
from rclpy.parameter import Parameter
import threading
import tempfile
import xml.etree.ElementTree as ET
from ikpy.chain import Chain

# Joint Orders for forward_position_controller
LA_ORDER = ["la_shoulder_pan_joint", "la_shoulder_lift_joint", "la_elbow_joint", "la_wrist_1_joint", "la_wrist_2_joint", "la_wrist_3_joint"]
RA_ORDER = ["ra_shoulder_pan_joint", "ra_shoulder_lift_joint", "ra_elbow_joint", "ra_wrist_1_joint", "ra_wrist_2_joint", "ra_wrist_3_joint"]

# Hand Presets
HAND_OPEN  = [0.0, -0.15, -0.15, -0.15, 0.0, -0.15, -0.15, -0.15, 0.0, -0.15, -0.15, -0.15, 0.50, -0.10, -0.10, -0.10]
HAND_CLOSE = [0.0, 1.60, 1.60, 1.20, 0.0, 1.60, 1.60, 1.20, 0.0, 1.60, 1.60, 1.20, 1.20, 1.10, 1.30, 1.20]

# Home Cartesian Poses (world frame)
LA_HOME_XYZRPY = [-0.360, 0.410, 0.996, -1.570, 0.003, 0.006]
RA_HOME_XYZRPY = [0.359, 0.295, 1.061, 0.0, 0.0, 0.0]

class HardwareIsolationTest(Node):
    def __init__(self):
        super().__init__('hardware_isolation_test')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

        # Publishers for Arms
        self.la_pub = self.create_publisher(Float64MultiArray, '/la_forward_position_controller/commands', 10)
        self.ra_pub = self.create_publisher(Float64MultiArray, '/ra_forward_position_controller/commands', 10)

        # Service Clients for Hands
        self.lh_client = self.create_client(SetDesPos, '/la_/setDesiredConfiguration')
        self.rh_client = self.create_client(SetDesPos, '/ra_/setDesiredConfiguration')

        # URDF and Joint State Cache
        self.urdf = None
        self.current_js = {}
        self.create_subscription(String, '/robot_description', self._urdf_cb, 10)
        self.create_subscription(JointState, '/ur_joint_states', self._js_cb, 10)

        self.get_logger().info("Waiting for URDF and Joint States...")
        
    def _urdf_cb(self, msg): self.urdf = msg.data
    def _js_cb(self, msg): 
        for n, p in zip(msg.name, msg.position): self.current_js[n] = p

    def call_hand(self, side, pos):
        client = self.lh_client if side == 'left' else self.rh_client
        if client.wait_for_service(timeout_sec=1.0):
            client.call_async(SetDesPos.Request(pos=pos))

    def move_arms(self, la_q, ra_q, duration=3.0):
        start_t = time.time()
        # Need to know current q to interpolate
        q0_la = [self.current_js.get(n, 0.0) for n in LA_ORDER]
        q0_ra = [self.current_js.get(n, 0.0) for n in RA_ORDER]
        
        while time.time() - start_t < duration:
            s = (time.time() - start_t) / duration
            smooth_s = 3*s**2 - 2*s**3
            cmd_la = [a + smooth_s*(b-a) for a, b in zip(q0_la, la_q)]
            cmd_ra = [a + smooth_s*(b-a) for a, b in zip(q0_ra, ra_q)]
            self.la_pub.publish(Float64MultiArray(data=cmd_la))
            self.ra_pub.publish(Float64MultiArray(data=cmd_ra))
            time.sleep(0.01)

    def solve_ik(self, side, xyzrpy):
        if not self.urdf: return None
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as f:
            f.write(self.urdf.encode())
            tmp_path = f.name
        
        base = f"{side[0]}a_base_link"
        tip = f"{side[0]}a_ee_link" if side == 'left' else f"{side[0]}a_tool0"
        chain = Chain.from_urdf_file(tmp_path, base_elements=[base])
        # Find index of tip
        names = [lk.name for lk in chain.links]
        # URDF joints are usually named differently, simplified for this test
        # We'll just use the target joint values if IK fails or is too complex for "simple" script
        # For this test, I will provide the pre-calculated Home and Home+10cm joints
        return None

    def run_test(self):
        # Home positions in radians (from baseline)
        la_home = [math.radians(x) for x in [-18.75, -138.64, -99.59, 57.97, 64.88, 137.67]]
        ra_home = [math.radians(x) for x in [27.33, -43.98, 98.18, -232.88, -71.98, 223.84]]
        
        # Approximate "Up 10cm" by offsetting the shoulder lift (index 1) and elbow (index 2)
        # This is a crude approximation for a hardware test without full IK dependency
        la_up = list(la_home); la_up[1] += 0.15; la_up[2] -= 0.1
        ra_up = list(ra_home); ra_up[1] += 0.15; ra_up[2] -= 0.1

        for i in range(10):
            self.get_logger().info(f"--- Cycle {i+1}/10 ---")
            
            # 1. Move Up
            self.get_logger().info("Moving Up...")
            self.move_arms(la_up, ra_up)
            
            # 2. Close/Open Hands
            self.get_logger().info("Grasping...")
            self.call_hand('left', HAND_CLOSE); self.call_hand('right', HAND_CLOSE)
            time.sleep(1.5)
            self.call_hand('left', HAND_OPEN); self.call_hand('right', HAND_OPEN)
            time.sleep(1.0)
            
            # 3. Move Down (Home)
            self.get_logger().info("Moving Down...")
            self.move_arms(la_home, ra_home)
            
            # 4. Close/Open Hands
            self.get_logger().info("Grasping...")
            self.call_hand('left', HAND_CLOSE); self.call_hand('right', HAND_CLOSE)
            time.sleep(1.5)
            self.call_hand('left', HAND_OPEN); self.call_hand('right', HAND_OPEN)
            time.sleep(1.0)

        self.get_logger().info("Test Complete.")

def main():
    rclpy.init()
    node = HardwareIsolationTest()
    
    # Wait for JS
    while rclpy.ok() and not node.current_js:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    threading.Thread(target=node.run_test, daemon=True).start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
