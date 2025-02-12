#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

class ForceVisualizer(Node):
    """
    力可视化节点
    用于在RViz中显示作用在小车上的控制力和地震力
    """
    def __init__(self):
        super().__init__('force_visualizer')
        
        # Create QoS profiles
        # 用于处理传感器数据的配置，允许数据丢失但保证实时性
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 尽力传输，允许丢包
            durability=DurabilityPolicy.VOLATILE,       # 不保存历史数据
            history=HistoryPolicy.KEEP_LAST,            # 只保留最新的数据
            depth=1                                     # 队列深度为1
        )
        
        # Subscribe to control force command
        self.force_sub = self.create_subscription(
            Float64,
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            self.control_force_callback,
            10)
            
        # Subscribe to earthquake force
        self.earthquake_sub = self.create_subscription(
            Float64,
            '/earthquake_force',  # We'll need to publish this from earthquake generator
            self.earthquake_force_callback,
            10)
            
        # Subscribe to joint states using the republished topic
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',  # Standard ROS2 joint states topic
            self.joint_state_callback,
            sensor_qos)
            
        # Publishers for force markers
        # 创建力的可视化标记发布器
        self.control_marker_pub = self.create_publisher(Marker, '/control_force_marker', 10)
        self.earthquake_marker_pub = self.create_publisher(Marker, '/earthquake_force_marker', 10)
        
        # 初始化状态变量
        self.cart_position = 0.0
        self.control_force = 0.0
        self.earthquake_force = 0.0
        self.get_logger().info('Force Visualizer node started')
        
    def joint_state_callback(self, msg):
        # Get cart position from joint states
        try:
            cart_idx = msg.name.index('cart_to_base')
            self.cart_position = msg.position[cart_idx]
            # Update both force visualizations when position changes
            # 当位置更新时，更新两个力的可视化
            self.publish_control_force()
            self.publish_earthquake_force()
        except ValueError:
            self.get_logger().warn('cart_to_base joint not found in joint states')
            
    def control_force_callback(self, msg):
        self.control_force = msg.data
        self.publish_control_force()
        
    def earthquake_force_callback(self, msg):
        self.earthquake_force = msg.data
        self.publish_earthquake_force()
        
    def create_force_marker(self, force, z_offset, is_control=True):
        """
        创建力的可视化箭头标记
        
        参数:
            force: 力的大小
            z_offset: 垂直方向的偏移量，用于区分不同的力
            is_control: 是否为控制力（控制力和地震力使用不同的颜色）
        """
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "control_force" if is_control else "earthquake_force"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set the start point at the cart's position
        start = Point()
        start.x = self.cart_position
        start.y = 0.0
        start.z = 0.15 + z_offset  # Offset vertically to show different forces
        
        # Set end point based on force magnitude (scaled for visualization)
        end = Point()
        scale = 0.1  # Scale factor to make force visible
        end.x = self.cart_position + force * scale
        end.y = 0.0
        end.z = 0.15 + z_offset
        
        marker.points = [start, end]
        
        # Set the arrow properties
        marker.scale.x = 0.04  # shaft diameter
        marker.scale.y = 0.1  # head diameter
        marker.scale.z = 0.1   # head length
        
        # Color based on force type and direction
        marker.color.a = 1.0
        if is_control:
            # Control force: red for positive, blue for negative
            if force > 0:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
        else:
            # Earthquake force: orange for positive, purple for negative
            if force > 0:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.5
                
        return marker
            
    def publish_control_force(self):
        marker = self.create_force_marker(self.control_force, 0.0, True)  # 控制力在下面
        self.control_marker_pub.publish(marker)
        
    def publish_earthquake_force(self):
        marker = self.create_force_marker(self.earthquake_force, 0.1, False)  # 
        self.earthquake_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS2（在创建节点之前调用）
    node = ForceVisualizer()  # 创建节点（执行__init__，设置订阅/发布等）
    rclpy.spin(node)  # 进入主循环，持续监听，即 while True，直到『ctrl+C or 发生错误』
    node.destroy_node()  # 清理资源，释放内存
    rclpy.shutdown()

if __name__ == '__main__':
    main() 