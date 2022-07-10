#!/usr/bin/python3
from unittest import result
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, Point

from std_srvs.srv import Empty
from turtlesim_interfaces.srv import RandGoal, SetGoal
from turtlesim_interfaces.action import GoToGoal

class FollowerInterface(Node):
    def __init__(self):
        super().__init__('follower_interface')
        self.enable_client = self.create_client(Empty,"/enable")
        self.set_goal_client = self.create_client(SetGoal,"/set_goal")
        self.notify_arrival_service = self.create_service(Empty,"/notify_arrival",self.notify_arrival_callback)
        self.go_to_goal_action_server = ActionServer(self, GoToGoal,'/go_to_goal',self.execute_callback)

    def execute_callback(self,goal_handle):
        init_time = self.get_clock().now()
        feedback_msg = GoToGoal.Feedback()
        self.isActive = True
        while self.isActive:#has not arrived
            dt = self.get_clock().now() - init_time
            feedback_msg.elasped_time = dt.nanoseconds/1e9
            goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        result = GoToGoal.Result()
        result.distance = 0.0
        return result


    def send_enable_request(self):
        req = Empty.Request()
        self.future = self.enable_client.call_async(req)

    def send_set_goal_request(self,position):
        req = SetGoal.Request()
        req.position  = position
        self.future = self.set_goal_client.call_async(req)

    def notify_arrival_callback(self,request,response):
        self.isActive = False
        return response



def main(args=None):
    rclpy.init(args=args)
    scheduler = FollowerInterface()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
