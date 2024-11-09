#!/usr/bin/env python3

# *******************************************************************************
# Script Name  : arm_control.py
# Author       : Daniel Sotelo Aguirre
# Date         : 08/11/2024
# Version      : v1.0
# *******************************************************************************

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class PIDController:
    def __init__(self, kp, ki, kd, joint_name):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.joint_name = joint_name

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.monotonic()

    def compute(self, error):
        current_time = time.monotonic()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            return 0.0
        
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error * delta_time
        i = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / delta_time
        d = self.kd * derivative

        # PID output
        output = p + i + d

        # Update previous error and time
        self.prev_error = error
        self.last_time = current_time

        return output
    
class JointPIDControllerNode(Node):
    def __init__(self):
        super().__init__('joint_pid_controller')

        # Parameters for wrist_1_joint (pitch) PID
        Kp_pitch = 0.00286
        Ki_pitch = 0.0
        Kd_pitch = 0.00012

        # Kp_pitch = 0.00017
        # Ki_pitch = 0.000017
        # Kd_pitch = 0.000222

        # Parameters for wrist_2_joint (yaw) PID
        Kp_yaw = 0.00286
        Ki_yaw = 0.0
        Kd_yaw = 0.00012
        # Kp_yaw = 0.0000981
        # Ki_yaw = 0.0000140
        # Kd_yaw = 0.0001525

        self.pid_pitch = PIDController(kp=Kp_pitch, ki=Ki_pitch, kd=Kd_pitch, joint_name='wrist_1_joint')
        self.pid_yaw = PIDController(kp=Kp_yaw, ki=Ki_yaw, kd=Kd_yaw, joint_name='wrist_2_joint')

        self.current_positions = {'wrist_1_joint': -2.094, 'wrist_2_joint': -1.57}

        # Publisher to send joint commands
        self.joint_command_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Subscription to ArUco marker offset topic
        self.offset_subscription = self.create_subscription(
            Int32MultiArray,
            "/aruco_offset",
            self.offset_callback,
            10
        )

        # Subscription to joint states to get current joint positions
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]
                # self.get_logger().info(f"Updated current position: {name}={msg.position[i]:.3f}")

    def offset_callback(self, msg):
        # Extract offset values from the message
        offset_x = msg.data[0]
        offset_y = msg.data[1]

        # self.get_logger().info(f"Received offset: x={offset_x}, y={offset_y}")

        # PID control for each joint based on offset
        pitch_adjustment = self.pid_pitch.compute(offset_y)
        yaw_adjustment = self.pid_yaw.compute(offset_x)

        # Calculate new positions
        new_pitch_position = self.current_positions['wrist_1_joint'] + pitch_adjustment
        new_yaw_position = self.current_positions['wrist_2_joint'] - yaw_adjustment
        # self.get_logger().info(f"Calculated new positions: wrist_1_joint={new_pitch_position:.3f}, wrist_2_joint={new_yaw_position:.3f}")

        # Create and publish joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['wrist_1_joint', 'wrist_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [new_pitch_position, new_yaw_position]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 400000000  # 500 ms for smooth updates

        trajectory_msg.points.append(point)
        # self.get_logger().info(f"Adjustments: wrist_1_joint={pitch_adjustment:.2f}, wrist_2_joint={yaw_adjustment:.2f}")
        
        self.joint_command_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointPIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)