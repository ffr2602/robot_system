#!/usr/bin/python3

import rclpy
import threading
import time
import math
import tf_transformations
import numpy as np

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from robot_system.robot import robot
from robot_system.can_setup import setup

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
        self.odom_publisher = self.create_publisher(Odometry, '/odom', qos_profile=qos_profile_services_default)
        self.odom_broadcaster = TransformBroadcaster(self, qos_profile_services_default)

        self.motor_vel = np.zeros(4).astype(int)
        self.robot = robot(self.GEOMETRI_ROBOT, self.WHEEL_RADIUS)
        self.can_setup = setup()

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
    
    def publish_odom(self, input):

        linear_x_velocity  = input[0]
        linear_y_velocity  = input[1]
        angular_z_velocity = input[2]

        current_time = time.time()
        delta_time = current_time - self.saved_time
        self.saved_time = current_time

        delta_x = delta_time * (linear_x_velocity * math.cos(self.angular_z_position) - linear_y_velocity * math.sin(self.angular_z_position))
        delta_y = delta_time * (linear_x_velocity * math.sin(self.angular_z_position) + linear_y_velocity * math.cos(self.angular_z_position))
        delta_z = delta_time * angular_z_velocity

        self.linear_x_position  += delta_x
        self.linear_y_position  += delta_y
        self.angular_z_position += delta_z

        tf_quat = tf_transformations.quaternion_from_euler(0, 0, self.angular_z_position)
        msg_quat = Quaternion(x=tf_quat[0], y=tf_quat[1], z=tf_quat[2], w=tf_quat[3])

        odom_transform = TransformStamped()
        odom_transform.header.stamp = self.get_clock().now().to_msg()
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_link'
        odom_transform.transform.translation.x = self.linear_x_position
        odom_transform.transform.translation.y = self.linear_y_position
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = msg_quat
        self.odom_broadcaster.sendTransform(odom_transform)

        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = self.linear_x_position
        odometry.pose.pose.position.y = self.linear_y_position
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation = msg_quat
        odometry.twist.twist.linear.x  = linear_x_velocity
        odometry.twist.twist.linear.y  = linear_y_velocity
        odometry.twist.twist.angular.z = angular_z_velocity
        self.odom_publisher.publish(odometry)

    def read_thread_function(self):
        while True:
            self.can_setup.send_data_can(self.motor_vel)
            twist_vel = self.robot.compute_velocity_robot_forward_kinematic(self.motor_vel)
            self.publish_wheels_state(self.robot.velocity_rad(self.motor_vel))
            self.publish_odom(twist_vel)



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

       

    
