#!/usr/bin/python3

import rclpy
import threading
import time
import math
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

from robot_system.robot import robot

class controller(Node):
    def __init__(self):
        super().__init__('controller_omni')
        self.get_logger().info('start node')

        chassis_length  = 1.004
        chassis_width   = 0.484
        wheel_thickness = 0.07
        wheel_radius    = 0.05

        wheel_offset_x = (chassis_length - (0.0795 * 2))/2
        wheel_offset_y = (chassis_width + wheel_thickness)/2

        self.saved_time = time.time()
        self.GEOMETRI_ROBOT = wheel_offset_x + wheel_offset_y
        self.WHEEL_RADIUS = wheel_radius
        self.CIRCLE_Length = 2 * self.WHEEL_RADIUS * math.pi
        self.RATIO_SPEED_WHEEL = (1 / (6000 * self.CIRCLE_Length)) / 0.817814436

        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.apply_velocity, qos_profile=qos_profile_system_default)
        self.joints_publisher = self.create_publisher(JointState, '/joint_states', qos_profile=qos_profile_sensor_data)

        self.motor_vel = np.zeros(4).astype(int)
        self.robot = robot(self.GEOMETRI_ROBOT, self.WHEEL_RADIUS)

        self.wheel_front_left_rotation  = 0.0
        self.wheel_front_right_rotation = 0.0
        self.wheel_rear_left_rotation   = 0.0
        self.wheel_rear_right_rotation  = 0.0

        self.linear_x_position  = 0.0
        self.linear_y_position  = 0.0
        self.angular_z_position = 0.0

        self.last_x_velocity = 0.0
        self.last_y_velocity = 0.0
        self.last_z_angular  = 0.0

        threading.Thread(target=self.read_thread_function).start()

    def apply_velocity(self, msg):
        input = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.motor_vel = self.robot.compute_velocity_robot_inverse_kinematic(input)
       
    def get_rotation_in_rad(self, wheel_rotation):
        return wheel_rotation % (2 * math.pi) - math.pi
    
    def publish_wheels_state(self, input):

        self.wheel_front_left_rotation  += input[0] * self.RATIO_SPEED_WHEEL 
        self.wheel_front_right_rotation += input[1] * self.RATIO_SPEED_WHEEL 
        self.wheel_rear_left_rotation   += input[2] * self.RATIO_SPEED_WHEEL 
        self.wheel_rear_right_rotation  += input[3] * self.RATIO_SPEED_WHEEL 

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_back_left_joint', 'wheel_back_right_joint']
        joint_states.position = [
            self.get_rotation_in_rad(self.wheel_front_left_rotation), 
            self.get_rotation_in_rad(self.wheel_front_right_rotation), 
            self.get_rotation_in_rad(self.wheel_rear_left_rotation), 
            self.get_rotation_in_rad(self.wheel_rear_right_rotation)]
        self.joints_publisher.publish(joint_states)

    def read_thread_function(self):
        while True:
            self.publish_wheels_state(self.robot.velocity_rad(self.motor_vel))


            
def main(args=None):
    rclpy.init(args=args)
    drive_controller = controller()
    try:
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
        pass
    except BaseException:
        print('Stopped by exception')
        raise
    finally:
        drive_controller.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()


       

    
