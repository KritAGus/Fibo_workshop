#!/usr/bin/python3
from turtlesim_control.base_controller import BaseController
import rclpy
from turtlesim.msg import Pose
import numpy as np

class TurtleFollower(BaseController):
    def __init__(self):
        super().__init__('turtle_follower')
        self.goal_subscription = self.create_subscription(Pose,'/goal',self.goal_callback,10)
        self.isEnable = True

    def goal_callback(self,msg):
        self.goal = np.array([msg.x, msg.y])
    
    def arrival_callback(self):
        self.isEnable = False
        self.get_logger().info('Finally Caught up !!')
    
    def departure_callback(self):
        self.isEnable = True
        self.get_logger().warning('Other turtle has moved !!')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleFollower()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
