#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class challenge_node(Node):
    def __init__(self):
        super().__init__('challenge_node')
        self.timer = self.create_timer(0.1, self.go_to)
        self.odom_sub = self.create_subscription(Odometry,'/odom', self.odom_callback,10)
        self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_pos = [(10.0,-6.0),(18.0, -0.2)]
        self.current_pos = self.target_pos[0]
        self.next_point = 0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.position = 0
        self.laser_data = []

        self.get_logger().info('Node has been started')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        # Get the yaw angle from the quaternion
        (_, _, self.yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def laser_callback(self, msg):
        # Store the laser scan data
        self.laser_data = msg.ranges

    def go_to(self):
        if not self.laser_data:
            return

        cmd_vel_msg = Twist()

        distance_x = self.current_pos[0] - self.x
        distance_y = self.current_pos[1] - self.y
        #self.get_logger().info('Distancia X: %.1f, Distancia Y: %.1f' % (distance_x, distance_y))
        theta = math.atan2(distance_y, distance_x)
        distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

        # Get the distance to the nearest obstacle in front of the robot
        front_distance = min(self.laser_data[50:155]) 
        
        if abs(distance_x) > 0.3 or abs(distance_y) > 0.3:
            if front_distance < 0.4:  
                cmd_vel_msg.linear.x = -0.5
                cmd_vel_msg.angular.z = 50.0  
            else:
                cmd_vel_msg.linear.x = 1.0 * distance  
                cmd_vel_msg.angular.z = 1.0 * (theta - self.yaw)
            self.publisher.publish(cmd_vel_msg)
        else:
            if self.next_point < (len(self.target_pos)-1):
                self.next_point += 1
                self.current_pos = self.target_pos[self.next_point]
                self.get_logger().info('Chegou no ponto alvo, proximo ponto: (%.1f, %.1f)' % (self.current_pos[0], self.current_pos[1]))
                self.get_logger().info('Posicao atual: (%.1f, %.1f)' % (self.x, self.y))
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.get_logger().info('Chegou no ponto final!')
                self.get_logger().info('Posicao atual: (%.1f, %.1f)' % (self.x, self.y))
                self.publisher.publish(cmd_vel_msg)
                self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = challenge_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
