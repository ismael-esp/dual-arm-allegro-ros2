#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from allegro_hand_interfaces.srv import SetDesPos
import numpy as np

class AllegroGloveController(Node):
    def __init__(self):
        super().__init__('allegro_glove_controller')

        # 1. Setup Service Client
        # Note: Ensure the namespace matches your 'slave:=True' launch output
        self.client = self.create_client(SetDesPos, '/la_/setDesiredConfiguration')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Allegro Hand Services (Ensure slave:=True)...')

        # 2. Setup Timer for Real-Time Loop (100Hz = 0.01s)
        self.timer_period = 0.01  
        self.timer = self.create_timer(self.timer_period, self.loop)
        
        self.get_logger().info('Glove Controller Started. Sending commands via Service.')

    def get_manus_glove_data(self):
        """
        PLACEHOLDER: Replace this with your actual Manus Glove mapping logic.
        Should return a list of 16 floats representing joint angles in radians.
        """
        # Example: Just sending zeros or a simple sine wave for testing
        t = self.get_clock().now().nanoseconds / 1e9
        angles = [0.0] * 16
        # angles[12] = 0.5 * np.sin(t) # Move the thumb roll
        return angles

    def loop(self):
        # Get your real-time mapped angles
        target_angles = self.get_manus_glove_data()

        # 3. Prepare Service Request
        request = SetDesPos.Request()
        request.pos = target_angles

        # 4. Asynchronous Call (Non-blocking)
        # This is critical for real-time performance. 
        # It sends the data and immediately continues the loop.
        self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = AllegroGloveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
