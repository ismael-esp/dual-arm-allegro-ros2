#!/usr/bin/env python3

import argparse
import json
import zmq
import rclpy
from rclpy.node import Node
from allegro_hand_interfaces.srv import SetDesPos

class DualServiceBridge(Node):
    def __init__(self, right_addr, left_addr, hz, clamp, lower, upper):
        super().__init__('allegro_dual_service_bridge')

        # 1. Initialize Service Clients
        # Using the namespaces provided in your examples: /ra_ and /la_
        self.cli_right = self.create_client(SetDesPos, '/ra_/setDesiredConfiguration')
        self.cli_left  = self.create_client(SetDesPos, '/la_/setDesiredConfiguration')

        # Wait for services (Slave Mode must be True on the driver)
        while not self.cli_right.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for RIGHT service (/ra_/setDesiredConfiguration)...')
        while not self.cli_left.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for LEFT service (/la_/setDesiredConfiguration)...')

        # 2. Memory Management: Track active futures to prevent RAM leaks
        self.future_right = None
        self.future_left = None

        # 3. ZMQ Setup
        self.ctx = zmq.Context.instance()
        self.sub_r = self.ctx.socket(zmq.SUB)
        self.sub_r.connect(right_addr)
        self.sub_r.setsockopt(zmq.SUBSCRIBE, b"")
        
        self.sub_l = self.ctx.socket(zmq.SUB)
        self.sub_l.connect(left_addr)
        self.sub_l.setsockopt(zmq.SUBSCRIBE, b"")

        self.poller = zmq.Poller()
        self.poller.register(self.sub_r, zmq.POLLIN)
        self.poller.register(self.sub_l, zmq.POLLIN)

        # 4. Control Loop (Timer)
        self.period = 1.0 / max(1e-6, hz)
        self.timer = self.create_timer(self.period, self._tick)

        self.clamp = clamp
        self.lower = lower
        self.upper = upper
        self.get_logger().info(f"Bridge started at {hz}Hz using SERVICES.")

    def _send_service_call(self, qpos, is_right):
        """Sends the position goal asynchronously if the previous call is done."""
        if self.clamp:
            qpos = [min(max(v, self.lower), self.upper) for v in qpos]

        request = SetDesPos.Request()
        request.pos = [float(v) for v in qpos]

        if is_right:
            # Only send if the previous call has finished to prevent RAM bloat
            if self.future_right is None or self.future_right.done():
                self.future_right = self.cli_right.call_async(request)
        else:
            if self.future_left is None or self.future_left.done():
                self.future_left = self.cli_left.call_async(request)

    def _handle_message(self, s):
        try:
            data = json.loads(s)
            qpos = data.get("qpos", None)
            hand = data.get("hand", None)

            if qpos is None or len(qpos) != 16:
                return

            if hand == "right":
                self._send_service_call(qpos, is_right=True)
            elif hand == "left":
                self._send_service_call(qpos, is_right=False)
        except Exception as e:
            self.get_logger().error(f"Mapping error: {e}")

    def _tick(self):
        # Poll ZMQ for new glove data
        events = dict(self.poller.poll(timeout=0))
        if self.sub_r in events:
            try:
                s = self.sub_r.recv_string(flags=zmq.NOBLOCK)
                self._handle_message(s)
            except zmq.Again:
                pass
        if self.sub_l in events:
            try:
                s = self.sub_l.recv_string(flags=zmq.NOBLOCK)
                self._handle_message(s)
            except zmq.Again:
                pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--right_addr', default='tcp://127.0.0.1:7777')
    parser.add_argument('--left_addr',  default='tcp://127.0.0.1:7778')
    parser.add_argument('--hz', type=float, default=60.0)
    parser.add_argument('--clamp', action='store_true')
    parser.add_argument('--lower', type=float, default=-0.5)
    parser.add_argument('--upper', type=float, default=1.6)
    args = parser.parse_args()

    rclpy.init()
    node = DualServiceBridge(args.right_addr, args.left_addr, args.hz, 
                             args.clamp, args.lower, args.upper)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
