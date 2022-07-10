#!/usr/bin/python3
import numpy as np
import rclpy
from turtlesim_control.base_controller import BaseController
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from turtlesim_interfaces.srv import SetGoal


class ViaPointFollower(BaseController):
    def __init__(self):
        super().__init__('Viapointfollower')
        self.set_goal_service = self.create_service(SetGoal,'/set_goal',self.set_goal_callback)
        self.enable_service = self.create_service(Empty,'/enable',self.enable_callback)
        self.notify_arrival_client = self.create_client(Empty,"/notify_arrival")

    def set_goal_callback(self,request,response):
        position = Point()
        position = request
        self.goal = np.array([request.position.x,request.position.y])
        return response
    def enable_callback(self,request,response):
        self.isEnable = True
        return response
    def send_notify_arrival_request(self):
        req = Empty.Request()
        self.future = self.notify_arrival_client.call_async(req)
    def arrival_callback(self):
        self.isEnable = False
        self.send_notify_arrival_request()
        self.get_logger().info('Arrived at via point !!')
    def departure_callback(self):
        self.isEnable = True
        self.get_logger().info('Goal Updated')

def main(args=None):
    rclpy.init(args=args)
    controller = ViaPointFollower()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
