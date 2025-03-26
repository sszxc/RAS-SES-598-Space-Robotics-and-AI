#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleControlMode, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from transform_utils import Transform, Rotation
from rich import print
from std_msgs.msg import String
import numpy as np
import re
np.set_printoptions(precision=4, suppress=True, linewidth=100)

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
            
        # Subscribe to geometry tracker outputs
        self.cylinder_pose_subscriber = self.create_subscription(
            Point, '/geometry/cylinder_center',
            self.cylinder_pose_callback, 10)
        self.cylinder_info_subscriber = self.create_subscription(
            Float32MultiArray, '/geometry/cylinder_info',
            self.cylinder_info_callback, 10)

        # subscribe to aruco marker pose
        self.aruco_pose_subscriber = self.create_subscription(
            String, '/aruco/marker_pose',
            self.aruco_pose_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.cylinder_position = {}
        self.cylinder_info = None
        
        # Flight parameters
        self.TARGET_HEIGHT = 5.0  # meters
        self.POSITION_THRESHOLD = 0.1  # meters
        
        # State machine
        self.state = "TAKEOFF"  # States: TAKEOFF, LAND
        
        # Create a timer to publish control commands
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.start_time = time.time()
        self.search_cycle_count = 0
        self.last_yaw = 0.0
        self.current_yaw = 0.0
        self.current_height = 0.0
        self.aruco_pose = 0, 0, 0


    def vehicle_odometry_callback(self, msg):
        """Store vehicle position from odometry."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Store vehicle status."""
        self.vehicle_status = msg

    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def is_at_target_height(self):
        """Check if the drone has reached the target height."""
        try:
            current_height = -self.vehicle_odometry.position[2]  # Convert NED to altitude
            return abs(current_height - self.TARGET_HEIGHT) < self.POSITION_THRESHOLD
        except (IndexError, AttributeError):
            return False

    def angle_is_close(self, angle1, angle2, threshold=30):
        """Check if two angles are close to each other."""
        # 将角度差值限制在 -pi 到 pi 之间
        diff = abs(angle1 - angle2) % (2 * np.pi)
        if diff > np.pi:
            diff = 2 * np.pi - diff
        # 如果threshold是以度为单位，需要转换为弧度
        return diff < threshold * np.pi / 180

    def cylinder_pose_callback(self, msg):
        """Store cylinder position from geometry tracker."""
        # self.cylinder_position = msg
        # if self.offboard_setpoint_counter % 10 == 0:  # Log every second
        #     self.get_logger().info(
        #         f'Cylinder at pixel ({msg.x:.1f}, {msg.y:.1f}) depth {msg.z:.2f}m'
        #     )
        if self.state == "SEARCH" or self.state == "CLIMB":
            # 寻找是否已经存在对应角度的数据
            history_cylinder_yaw = [yaw for yaw in self.cylinder_position.keys()]
            for yaw in history_cylinder_yaw:
                if self.angle_is_close(yaw, self.current_yaw):
                    self.cylinder_position[yaw] = [msg.z, self.current_height, self.search_cycle_count]
                    # self.get_logger().info(f"Update cylinder {yaw*180/np.pi:.3f}°")
                    self.print_cylinder_position()
                    return
            self.cylinder_position[self.current_yaw] = [msg.z, self.current_height, self.search_cycle_count]
            self.print_cylinder_position()
            # self.get_logger().info(f"New cylinder {self.current_yaw} at {self.current_height:.2f}m")

    def print_cylinder_position(self):
        _string = ""
        for yaw, info in self.cylinder_position.items():
            _string += f"C{yaw*180/np.pi:.3f}°: d {info[0]:.2f}m h {info[1]:.2f}m cycle {info[2]}. "
        self.get_logger().info(_string)

    def cylinder_info_callback(self, msg):
        """Store cylinder information from geometry tracker."""
        self.cylinder_info = msg.data  # [width, height, angle, confidence]
        if self.offboard_setpoint_counter % 10 == 0:  # Log every second
            self.get_logger().info(
                f'Cylinder size: {msg.data[0]:.1f}x{msg.data[1]:.1f} '
                f'angle: {msg.data[2]:.1f}° confidence: {msg.data[3]:.2f}'
            )

    def aruco_pose_callback(self, msg):
        """Store aruco marker pose from aruco tracker."""
        pattern = r'x:([\d.-]+)m, y:([\d.-]+)m, z:([\d.-]+)m'
        matches = re.search(pattern, msg.data)
        if matches:
            x = float(matches.group(1))
            y = float(matches.group(2))
            z = float(matches.group(3))
            self.aruco_pose = x, y, z

            current_pos = self.vehicle_odometry.position
            aruco_offset_Fcam = self.aruco_pose[0:2]
            cos_yaw = np.cos(self.current_yaw)
            sin_yaw = np.sin(self.current_yaw)
            aruco_offset_Fworld = np.array([
                aruco_offset_Fcam[0] * sin_yaw - aruco_offset_Fcam[1] * cos_yaw,
                aruco_offset_Fcam[0] * cos_yaw + aruco_offset_Fcam[1] * sin_yaw,
                self.aruco_pose[2],
            ])
            self.aruco_guide_target = current_pos + aruco_offset_Fworld * 0.3

        self.get_logger().info(f"Aruco marker pose: {self.aruco_guide_target}")

    def control_loop(self):
        """Timer callback for control loop."""
        if self.offboard_setpoint_counter == 10:  # 计数器
            self.engage_offboard_mode()  # 切换到offboard模式，允许后面的起飞命令
            self.arm()  # 解锁无人机
            self.get_logger().info("Vehicle armed and offboard mode enabled")

        self.publish_offboard_control_mode()

        try:
            self.current_height = -self.vehicle_odometry.position[2]  # 获取当前高度
        except (IndexError, AttributeError):
            self.current_height = 0.0

        if self.state == "TAKEOFF":
            # Take off to target height
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.TARGET_HEIGHT,  # Negative because PX4 uses NED -> North-East-Down 坐标系
                yaw=0.0
            )
            
            # Log current height every second
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Taking off... Current height: {self.current_height:.2f}m")
            
            # Check if we've reached target height
            if self.is_at_target_height():
                self.state = "SEARCH"
                self.get_logger().info(f"Reached target height of {self.TARGET_HEIGHT}m, beginning searching")

        elif self.state == "SEARCH":
            # directly to land - only forquick test
            # current_pos = self.vehicle_odometry.position
            # target_pos = np.array([5.0, 0.0, -15.0])
            # distance_to_target = np.linalg.norm(target_pos - current_pos)
            # self.get_logger().info(f"current_pos: {current_pos}, target_pos: {target_pos}, distance_to_target: {distance_to_target:.2f}m")
            # if distance_to_target < 0.1:
            #     self.get_logger().info(f"Ready to land")
            #     self.state = "LAND"
            # self.publish_trajectory_setpoint(
            #     x=5.0,
            #     y=0.0,
            #     z=-15.0,  # Negative because PX4 uses NED -> North-East-Down 坐标系
            #     yaw=0.0
            # )
            # return
            current_pose = Transform(Rotation.from_quat(self.vehicle_odometry.q, scalar_first=True),
                                     self.vehicle_odometry.position)
            # self.get_logger().info(f"Current rpy: {current_pose.rotation.as_euler('xyz', degrees=True)}°")
            self.current_yaw = current_pose.rotation.as_euler('xyz', degrees=False)[2]
            self.target_height = self.current_height + 0.15

            # target_yaw_list = [0.0, np.pi/2, np.pi, -np.pi/2]
            # current_time = time.time() - self.start_time
            # print(f"current_time: {current_time}")
            # target_yaw = target_yaw_list[int(current_time/3) % len(target_yaw_list)]

            # continuously rotate
            target_yaw = self.current_yaw + 0.3
            if target_yaw > np.pi:
                target_yaw = target_yaw - 2*np.pi

            if self.last_yaw < 0 and self.current_yaw > 0:
                print(f'last_yaw: {self.last_yaw}, current_yaw: {self.current_yaw}')
                self.search_cycle_count += 1
                if self.search_cycle_count > 1:  # or True:  # TODO for testing
                    # passed a full circle, check if only one cylinder is in view
                    cylinders_last_seen_cycle = [info[2] for info in self.cylinder_position.values()]
                    self.get_logger().info(f"Checking if only one cylinder is in view\nlast seen cycle: {cylinders_last_seen_cycle}, current cycle: {self.search_cycle_count}")
                    if len([c for c in cylinders_last_seen_cycle if c == self.search_cycle_count-1]) == 1:
                        index = cylinders_last_seen_cycle.index(self.search_cycle_count-1)
                        self.get_logger().info(f"Only one cylinder is in view, go climbing")
                        self.state = "CLIMB"
                        self.target_cylinder = list(self.cylinder_position.keys())[index]
            self.last_yaw = self.current_yaw

            # test aroung 0°
            # target_yaw = self.current_yaw + 0.01
            # if target_yaw < - 0.3 or target_yaw > + 0.3:
            #     target_yaw = - 0.3

            # self.get_logger().info(f"Current yaw: {self.current_yaw/np.pi*180:.2f}°, target yaw: {target_yaw/np.pi*180:.2f}°")
            # self.get_logger().info(f"Current yaw: {self.current_yaw/np.pi*180:.2f}°, current height: {self.current_height:.2f}m")
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.target_height,
                yaw=target_yaw
            )
            
            # Search for the cylinder
            # if _cylinder_in_view:
            #     self.last_cylinder_yaw = self.current_yaw
            #     self.last_cylinder_height = current_height

        elif self.state == "CLIMB":
            self.get_logger().info(f"Climbing to {self.target_cylinder}°")
            target_yaw = self.target_cylinder + np.random.uniform(-0.2, 0.2)
            self.target_height = self.current_height + 0.3

            # check if the target is missing
            if abs(self.cylinder_position[self.target_cylinder][1] - self.current_height) > 1.0:
                self.get_logger().info(f"Target height: {self.cylinder_position[self.target_cylinder][1]:.2f}m, current height: {self.current_height:.2f}m")
                self.get_logger().info(f"Target {self.target_cylinder}° is missing, go approaching")
                self.state = "APPROACH"
                self.target_x = np.cos(self.target_cylinder) * (self.cylinder_position[self.target_cylinder][0] + 0.03)
                self.target_y = np.sin(self.target_cylinder) * (self.cylinder_position[self.target_cylinder][0] + 0.03)
                self.target_height = self.current_height + 1.5
                print(f"target_x: {self.target_x:.2f}, target_y: {self.target_y:.2f}, target_height: {self.target_height:.2f}")

            # self.target_height += 0.01
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.target_height,
                yaw=target_yaw
            )

        elif self.state == "APPROACH":
            current_pose = Transform(Rotation.from_quat(self.vehicle_odometry.q, scalar_first=True),
                                     self.vehicle_odometry.position)
            target_pos = np.array([self.target_x, self.target_y, -self.target_height])
            distance_to_target = np.linalg.norm(target_pos - current_pose.translation)
            # self.get_logger().info(f"target_pos: {target_pos}, current_pos: {current_pose.translation}")
            # self.get_logger().info(f"Approaching {self.target_cylinder}°, distance to target: {distance_to_target:.2f}m")
            if distance_to_target < 0.1:
                self.get_logger().info(f"Ready to land")
                self.state = "LAND"

            self.publish_trajectory_setpoint(
                x=self.target_x,
                y=self.target_y,
                z=-self.target_height,
                yaw=self.target_cylinder
            )

        elif self.state == "LAND":
            # Land by going to height 0

            if 'aruco_guide_target' in self.__dict__:
                self.target_height = self.current_height - 0.2
                self.publish_trajectory_setpoint(
                    x=self.aruco_guide_target[0],
                    y=self.aruco_guide_target[1],
                    z=-self.target_height,
                    yaw=0.0
                )
            
            # Log current height every second
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Landing... Current height: {self.current_yaw:.2f}m")

        self.offboard_setpoint_counter += 1

def main():
    rclpy.init()
    node = SimpleTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 