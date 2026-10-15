#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class UrJointPublishNode(Node):
    def __init__(self):
        super().__init__('joint_republisher_left')
        
        self.sub=self.create_subscription(
            JointState,
            '/joint_states_left_hand',
            self.callback,
            10
            )
        
        self.pub=self.create_publisher(JointState,'/joint_states_re_left',1)
        
        # #r
        self.mapping={
            "la_shoulder_pan_joint":"la_shoulder_pan_joint",
            "la_shoulder_lift_joint":"la_shoulder_lift_joint",
            "la_elbow_joint":"la_elbow_joint",
            "la_wrist_1_joint":"la_wrist_1_joint",
            "la_wrist_2_joint":"la_wrist_2_joint",
            "la_wrist_3_joint":"la_wrist_3_joint",
        }
        self.output_order = [
            "la_shoulder_lift_joint",
            "la_elbow_joint",
            "la_wrist_1_joint",
            "la_wrist_2_joint",
            "la_wrist_3_joint",
            "la_shoulder_pan_joint",
        ]
        #s
        # self.mapping={
        #     "la_shoulder_pan_joint":"shoulder_pan_joint",
        #     "la_shoulder_lift_joint":"shoulder_lift_joint",
        #     "la_elbow_joint":"elbow_joint",
        #     "la_wrist_1_joint":"wrist_1_joint",
        #     "la_wrist_2_joint":"wrist_2_joint",
        #     "la_wrist_3_joint":"wrist_3_joint",
        # }
        # self.output_order = [
        #     "shoulder_lift_joint",
        #     "elbow_joint",
        #     "wrist_1_joint",
        #     "wrist_2_joint",
        #     "wrist_3_joint",
        #     "shoulder_pan_joint",
        # ]
        
    def callback(self, msg_in: JointState):
        msg_out = JointState()
        msg_out.header.stamp = msg_in.header.stamp
        msg_out.header.frame_id = 'base_link'


        data_dict = {}
        for i, name in enumerate(msg_in.name):
            if name in self.mapping:
                mapped_name = self.mapping[name]
                data_dict[mapped_name] = {
                    "pos": msg_in.position[i],
                    "vel": msg_in.velocity[i] if len(msg_in.velocity) > i else 0.0,
                    "eff": msg_in.effort[i] if len(msg_in.effort) > i else 0.0,
                }


        for joint in self.output_order:
            if joint in data_dict:
                msg_out.name.append(joint)
                msg_out.position.append(data_dict[joint]["pos"])
                msg_out.velocity.append(data_dict[joint]["vel"])
                msg_out.effort.append(data_dict[joint]["eff"])

        self.pub.publish(msg_out)
    
def main(args=None):
    rclpy.init(args=args)
    node=UrJointPublishNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
        