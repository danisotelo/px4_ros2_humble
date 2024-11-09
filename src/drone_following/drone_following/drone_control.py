#!/usr/bin/env python3

# *******************************************************************************
# Script Name  : drone_control.py
# Author       : Daniel Sotelo Aguirre
# Date         : 08/11/2024
# Version      : v1.0
# *******************************************************************************

import rclpy
import random
import math
import itertools
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint',
              qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command', 
            qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, 
            qos_profile)
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0
        self.offboardMode = False
        self.last_waypoint = [0, 0, 0]
        self.selected_waypoint = [0, 0, 0]

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.waypoint_timer = self.create_timer(6.0, self.waypoint_callback)

        # Generate four random waypoints initially
        self.waypoints = self.generate_random_waypoints(4)

    def generate_random_waypoints(self, count):
        """Generate a list of random waypoints"""
        waypoints = []

        # Define space boundaries
        x_min, x_max = -1.0,  1.0
        y_min, y_max = -0.5,  0.5
        z_min, z_max =   -2, -1.5

        # Determine number of divisions along x and z axes
        x_divisions = int(math.sqrt(count))
        z_divisions = count // x_divisions

        # Step sizes for each division
        x_step = (x_max - x_min) / x_divisions
        z_step = (z_max - z_min) / z_divisions

        # Generate waypoints across each grid cell
        for i, j in itertools.product(range(x_divisions), range(z_divisions)):
            # Calculate region boundaries for this grid cell
            x_region_min = x_min + i * x_step
            x_region_max = x_region_min + x_step
            z_region_min = z_min + j * z_step
            z_region_max = z_region_min + z_step

            # Randomly select a point within this region
            x = random.uniform(x_region_min, x_region_max)
            y = random.uniform(y_min, y_max)
            z = random.uniform(z_region_min, z_region_max)

            position = [x, y, z]
            waypoints.append(position)

        return waypoints
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = -1.5708
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints [{x:.2f}, {y:.2f}, {z:.2f}]")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > (self.takeoff_height + 0.5) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height)
            self.offboardMode = True

        elif self.vehicle_local_position.z <= -6.0:
            self.land()
            self.disarm()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def waypoint_callback(self) -> None:
        """Callback function for the waypoint timer"""
        if self.offboardMode == True:
            self.selected_waypoint = random.choice(self.waypoints)
            while self.selected_waypoint == self.last_waypoint:
                self.selected_waypoint = random.choice(self.waypoints)
            
            self.publish_position_setpoint(self.selected_waypoint[0], self.selected_waypoint[1], self.selected_waypoint[2])
            self.last_waypoint = self.selected_waypoint


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)