#!/usr/bin/env python3 

import random 
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Kill, Spawn

from functools import partial 



class TurtleKillerNode(Node):
    def __init__(self):
        super().__init__("Game")

        self.get_logger().info("Turtle controller has been started")

        self.ONE_cmd_vel_publisher_ = self.create_publisher(Twist, "/ONE/cmd_vel", 10)
        self.ONE_pose_subscriber_ = self.create_subscription(Pose, "/ONE/pose", partial(self.callback_pose, "ONE"), 10)

        self.TWO_cmd_vel_publisher_ = self.create_publisher(Twist, "/TWO/cmd_vel", 10)
        self.TWO_pose_subscriber_ = self.create_subscription(Pose, "/TWO/pose", partial(self.callback_pose, "TWO"), 10)
        
        self.THREE_cmd_vel_publisher_ = self.create_publisher(Twist, "/THREE/cmd_vel", 10)
        self.THREE_pose_subscriber_ = self.create_subscription(Pose, "/THREE/pose", partial(self.callback_pose, "THREE"), 10)

        self.FOUR_cmd_vel_publisher_ = self.create_publisher(Twist, "/FOUR/cmd_vel", 10)
        self.FOUR_pose_subscriber_ = self.create_subscription(Pose, "/FOUR/pose", partial(self.callback_pose, "FOUR"), 10)


        self.call_kill_service("turtle1")
        self.call_spawn_service("ONE", 1.0, 1.0, 255, 0, 0)
        self.call_spawn_service("TWO", 4.0, 1.0, 0, 255, 0)
        self.call_spawn_service("THREE", 7.0,1.0, 0, 0, 255)
        self.call_spawn_service("FOUR", 10.0, 1.0, 255, 255, 255)
        
        self.get_logger().info("All turtles at line!")



    def callback_pose(self, turtle_name, pose: Pose):
        cmd = Twist()
        speed = random.random()*4
        cmd.linear.x = float(speed)
        cmd.angular.z = 0.0

        if turtle_name == "ONE":
            self.ONE_cmd_vel_publisher_.publish(cmd)
        elif turtle_name == "TWO":
            self.TWO_cmd_vel_publisher_.publish(cmd)
        elif turtle_name == "THREE":
            self.THREE_cmd_vel_publisher_.publish(cmd)
        elif turtle_name == "FOUR":
            self.FOUR_cmd_vel_publisher_.publish(cmd)

        if pose.y>11 :
            self.get_logger().info(f"Turtle {turtle_name} has finished the race!")
            self.call_kill_service(turtle_name)
        
    # kill service 
    def call_kill_service(self, turtle_name):
        kill_client = self.create_client(Kill, "/kill")
        while not kill_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for kill service")
        kill_request = Kill.Request()
        kill_request.name= turtle_name
        kill_future = kill_client.call_async(kill_request)
        kill_future.add_done_callback(partial(self.kill_callback))

    def kill_callback(self, kill_future):
        try:
            kill_response = kill_future.result()
        except Exception as e:
            self.get_logger().error("Kill Service call failed: %r" %(e, ))

    # spawn service 
    def call_spawn_service(self, name, x, y, r, g, b):
        spawn_client = self.create_client(Spawn, '/spawn') 
        while not spawn_client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for spawn service')
        spawn_request = Spawn.Request()
        spawn_request.x = x
        spawn_request.y = y
        spawn_request.theta = 1.5708
        spawn_request.name = name
        spawn_future = spawn_client.call_async(spawn_request)
        spawn_future.add_done_callback(partial(self.callback_spawn,name = name, r = r, g = g, b = b))
    
    def callback_spawn (self, spawn_future, name, r, g, b):
        try:
            spawn_response = spawn_future.result()
            if spawn_response.name == name:
                self.call_set_pen_service(name, r, g, b)
            else: 
                self.get_logger().error("Spawn failed for %s" %name)
        except Exception as e:
            self.get_logger().error("Spawn Service call failed: %r" %(e, ))

    # set pen service 
    def call_set_pen_service(self, name, r, g, b, width = 3, off= 0):
        set_pen_client = self.create_client(SetPen, f'/{name}/set_pen')
        while not set_pen_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for set pen service")
        
        set_pen_request = SetPen.Request()
        set_pen_request.r = r
        set_pen_request.b = b
        set_pen_request.g = g
        set_pen_request.width = width
        set_pen_request.off = off

        set_pen_future = set_pen_client.call_async(set_pen_request)
        set_pen_future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen (self, set_pen_future):
        try:
            set_pen_response = set_pen_future.result()
        except Exception as e:
            self.get_logger().error("Set Pen Service call failed: %r" %(e, ))


def main (args = None):
    rclpy.init(args = args)
    node = TurtleKillerNode()
    rclpy.spin(node)
    rclpy.shutdown()
