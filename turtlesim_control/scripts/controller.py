#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from std_srvs.srv import Empty

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.command_publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pose_subscription = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
        self.pose = Pose()
        self.rand_goal_service = self.create_service(Empty,'/rand_goal',self.rand_goal_callback)
        self.goal = np.array([2.0,3.0])

    def timer_callback(self):
        msg = self.control()
        self.command_publisher.publish(msg)
    def pose_callback(self,msg):
        self.pose = msg
    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = 3.0
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.1:
            v = 1.0
        else:
            v = 0.0

        msg.linear.x = v
        msg.angular.z = w
        return msg
    def rand_goal_callback(self,request,response):
        self.goal = 9*np.random.rand(2)+0.5
        self.get_logger().info(f'New goal => x:{self.goal[0]},y:{self.goal[1]}')
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
