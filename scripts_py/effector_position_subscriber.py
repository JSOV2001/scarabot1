#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class EffectorPositionSubscriber(Node):
    def __init__(self):
        super().__init__("effector_position_subscriber")
        self.sample = 0
        effector_position_subscriber = self.create_subscription(
            Pose, 
            '/effector_position', 
            self.effector_position_callback, 
            10
        )

    def effector_position_callback(self, effector_position_msg):
        h_x = effector_position_msg.position.x
        h_y = effector_position_msg.position.y
        h_z = effector_position_msg.position.z

        print(f"\nCurrent Time: {round(self.sample, 2)} secs")
        print("Current Effector Position:")
        print(f"h_x: {round(h_x, 2)} m")
        print(f"h_y: {round(h_y, 2)} m")
        print(f"h_z: {round(h_z, 2)} m")

        self.sample = self.sample + 0.1

def main():
    rclpy.init()
    try:
        effector_position_subscriber_node = EffectorPositionSubscriber()
        rclpy.spin(effector_position_subscriber_node)
    except:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
