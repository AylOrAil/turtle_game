#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from turtlesim.srv import Kill

from functools import partial 

class TurtleKillerNode(Node):
    def __init__(self):
        super().__init__("Game")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle controller has been started")

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.1
        
        self.cmd_vel_publisher_.publish(cmd)

        if pose.x>9 :
            self.get_logger().info("Turtle Killed!")
            self.kill_turtle("turtle1")
        

    def kill_turtle(self, turtle_name):
        kill_client = self.create_client(Kill, "/kill")
        while not kill_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service")
        kill_request = Kill.Request()
        kill_request.name= turtle_name
        kill_future = kill_client.call_async(kill_request)
        kill_future.add_done_callback(partial(self.kill_call_back))

    def kill_call_back(self, kill_future):
        try:
            kill_response = kill_future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e, ))

def main (args = None):
    rclpy.init(args = args)
    node = TurtleKillerNode()
    rclpy.spin(node)
    rclpy.shutdown()
