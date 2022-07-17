#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from math import sin, cos
from turtlesim.msg import Pose


class FramePublisher(Node):

    def __init__(self):
        super().__init__(sys.argv[1]+'_tf2_Broadcaster')

        # Declare and acquire `turtlename` parameter
        self.declare_parameter('turtlename', sys.argv[1] )
        self.turtlename = self.get_parameter(
            'turtlename').get_parameter_value().string_value
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)


        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.turtlename
        t.child_frame_id = 'base_footprint'
        self.static_broadcaster.sendTransform(t)


        
        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.subscription

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0, 0, 0, 0]
        q[0] = (cy * cp * cr) + (sy * sp * sr)
        q[1] = (cy * cp * sr) - (sy * sp * cr)
        q[2] = (sy * cp * sr) + (cy * sp * cr)
        q[3] = (sy * cp * cr) - (cy * sp * sr)

        return q

    def handle_turtle_pose(self, msg): #boardcast
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        t.transform.translation.x = msg.x 
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = self.quaternion_from_euler(0,0,msg.theta)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.broadcaster.sendTransform(t)
        pass

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()