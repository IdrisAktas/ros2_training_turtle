#!/usr/bin/env python3

import sys
import math
from functools import partial   # Callback'e ekstra parametre geçirmek için

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim_interfaces.msg import TurtleArray  
from geometry_msgs.msg import Twist
from turtlesim_interfaces.srv import CatchTurtle

class FindGoToLocationNode(Node):

    def __init__(self):
        super().__init__('find_go_to_location_node')
        self.coeff=1.5
        self.pose_threshold = 0.3
        self.pose_threshold_angular = 0.03
        self.target_x = 4.0
        self.target_y = 9.0


        self.pose=None
        self.new_turtle_to_catch_=None



        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback_turtle_pose, 10)
        self.new_turtle_subscriber_ = self.create_subscription(TurtleArray, '/new_turtles', self.callback_new_turtles, 10)

        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.turtle_controller)


        self.get_logger().info("Find Go To Location Node has been started!")




    def callback_turtle_pose(self, msg):
        self.pose = msg



    def callback_new_turtles(self, msg):
        if len(msg.turtles) > 0:
         self.new_turtle_to_catch_ = msg.turtles[0]

    def turtle_controller(self):
        if self.pose==None or self.new_turtle_to_catch_==None:
            return        
        
        msg = Twist()

        dist_x = self.target_x - self.pose.x
        dist_y = self.target_y - self.pose.y

        distance = math.sqrt(dist_x**2 + dist_y**2)

        target_theta = math.atan2(dist_y, dist_x)

        angle_error = target_theta - self.pose.theta

        if abs(angle_error) > self.pose_threshold_angular:
            msg.angular.z = angle_error
            msg.linear.x = 0.0

        else:
            if distance >= self.pose_threshold:
                msg.linear.x = distance    # distance*self.coeff
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.call_cath_turtle_server(self.new_turtle_to_catch_.name)
                self.new_turtle_to_catch_=None
                self.get_logger().info("SUCCESS! Target Reached!")

        self.publisher_.publish(msg)



    def call_cath_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "/catch_turtle")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server - [Catch Turtles]")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle,turtle_name=turtle_name))



    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            self.get.logger().info(f"Turtle {turtle_name} caught successfully.")


        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))





def main(args=None):
    rclpy.init(args=args)
    node = FindGoToLocationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
