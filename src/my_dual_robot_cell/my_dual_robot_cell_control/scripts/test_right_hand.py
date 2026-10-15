#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from allegro_hand_interfaces.srv import SetDesPos
import time

class RightHandTester(Node):
    def __init__(self):
        super().__init__('right_hand_tester')
        
        # Initialize Service Client for the Right Hand
        self.client = self.create_client(SetDesPos, '/ra_/setDesiredConfiguration')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Right Hand service (/ra_/setDesiredConfiguration)...')

        # Define Poses (Radians)
        self.open_pose = [0.0] * 16
        
        # Partially closed pose to avoid excessive current draw while testing
        self.close_pose = [
            0.0, 1.0, 1.1, 0.4,  # Index
            0.0, 1.0, 1.1, 0.4,  # Middle
            0.0, 1.0, 1.1, 0.4,  # Ring
            1.1, 0.8, 0.9, 0.4   # Thumb
        ]
        
        # Moving pose (individual finger wiggle)
        self.move_pose = [
            0.0, 0.5, 0.5, 0.0,  # Index mid-flex
            0.0, 0.0, 0.0, 0.0,  # Middle open
            0.0, 0.5, 0.5, 0.0,  # Ring mid-flex
            0.5, 0.2, 0.2, 0.0   # Thumb mid-flex
        ]

    def send_pose(self, pose):
        if not rclpy.ok():
            return
        request = SetDesPos.Request()
        request.pos = [float(p) for p in pose]
        self.get_logger().info(f'Sending pose command...')
        self.client.call_async(request)

    def run_test(self):
        start_time = time.time()
        duration = 1000.0  # 30 seconds test
        
        self.get_logger().info('--- Starting 30 Second Right Hand Test ---')
        
        try:
            while (time.time() - start_time) < duration and rclpy.ok():
                # 1. Open
                self.get_logger().info('Action: Opening Hand')
                self.send_pose(self.open_pose)
                time.sleep(2.0)

                # 2. Wiggle/Move
                self.get_logger().info('Action: Moving Fingers')
                self.send_pose(self.move_pose)
                time.sleep(2.0)

                # 3. Close
                self.get_logger().info('Action: Closing Hand')
                self.send_pose(self.close_pose)
                time.sleep(2.0)
                
                remaining = duration - (time.time() - start_time)
                self.get_logger().info(f'Time remaining: {max(0.0, remaining):.1f}s')

            self.get_logger().info('Action: Returning to Neutral')
            self.send_pose(self.open_pose)
            self.get_logger().info('--- Test Complete ---')

        except KeyboardInterrupt:
            self.get_logger().info('Test interrupted by user')

def main(args=None):
    rclpy.init(args=args)
    node = RightHandTester()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
