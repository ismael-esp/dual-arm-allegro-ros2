#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from allegro_hand_interfaces.srv import SetDesPos
import time

class AllegroToggle(Node):
    def __init__(self):
        super().__init__('allegro_toggle_node')
        
        # 1. Initialize Service Client
        self.client = self.create_client(SetDesPos, '/ra_/setDesiredConfiguration')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service... Ensure slave:=True')

        # 2. Define Poses (Radians)
        # Sequence: [Index(4), Middle(4), Ring(4), Thumb(4)]
        self.open_pose = [0.0] * 16
        
        # Close pose: Flexing the proximal and middle joints (j1 and j2 of each finger)
        # We keep the thumb slightly offset to avoid hitting the palm
        self.close_pose = [
            0.0, 1.2, 1.4, 0.5,  # Index
            0.0, 1.2, 1.4, 0.5,  # Middle
            0.0, 1.2, 1.4, 0.5,  # Ring
            1.1, 1.0, 1.0, 0.5   # Thumb
        ]

    def send_pose(self, pose):
        request = SetDesPos.Request()
        request.pos = pose
        self.client.call_async(request)

    def run(self):
        try:
            while rclpy.ok():
                self.get_logger().info('Opening Hand...')
                self.send_pose(self.open_pose)
                time.sleep(3.0)

                self.get_logger().info('Closing Hand...')
                self.send_pose(self.close_pose)
                time.sleep(3.0)
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AllegroToggle()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
