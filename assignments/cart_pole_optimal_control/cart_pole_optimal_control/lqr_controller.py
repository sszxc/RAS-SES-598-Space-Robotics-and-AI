#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
        
        # State space matrices
        # x' = Ax + Bu
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
        
        # LQR cost matrices
        self.Q = np.diag([1.0, 1.0, 10.0, 10.0])  # State cost
        self.R = np.array([[0.1]])  # Control cost
        
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        
        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force', 
            10
        )
        
        # Verify publisher created successfully
        if self.cart_cmd_pub:
            self.get_logger().info('Force command publisher created successfully')
        
        # 每当仿真器发布新的关节状态时，这个回调函数就触发，更新 self.x
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )
        
        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)

        # 创建状态发布器
        self.cart_pos_pub = self.create_publisher(Float64, '/plot/cart_position', 10)
        self.cart_vel_pub = self.create_publisher(Float64, '/plot/cart_velocity', 10)
        self.pole_angle_pub = self.create_publisher(Float64, '/plot/pole_angle', 10)
        self.pole_vel_pub = self.create_publisher(Float64, '/plot/pole_velocity', 10)
        self.control_force_pub = self.create_publisher(Float64, '/plot/control_force', 10)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.stop_control_flag = False

        self.get_logger().info('Cart-Pole LQR Controller initialized')
    
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
    
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        # self.get_logger().info('控制器从Gazebo获取更新的关节位置')

        try:
            # Get indices for cart and pole joints
            cart_idx = msg.name.index('cart_to_base')  # Cart position/velocity
            pole_idx = msg.name.index('pole_joint')    # Pole angle/velocity
            
            # State vector: [x, ẋ, θ, θ̇]
            self.x = np.array([
                [msg.position[cart_idx]],     # Cart position (x)
                [msg.velocity[cart_idx]],     # Cart velocity (ẋ)
                [msg.position[pole_idx]],     # Pole angle (θ)
                [msg.velocity[pole_idx]]      # Pole angular velocity (θ̇)
            ])
            
            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={msg.position[cart_idx]:.3f}, cart_vel={msg.velocity[cart_idx]:.3f}, pole_angle={msg.position[pole_idx]:.3f}, pole_vel={msg.velocity[pole_idx]:.3f}')
                self.state_initialized = True
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}, msg={msg.name}')
    
    def control_loop(self):
        """Compute and apply LQR control."""
        if self.stop_control_flag:
            return
        try:
            if not self.state_initialized:
                self.get_logger().warn('State not initialized yet')
                return

            # 检查小车位置是否超出限制
            cart_position = float(self.x[0][0])
            position_limit = 2.5
            if abs(cart_position) > position_limit:
                self.get_logger().warn(f'小车位置 ({cart_position:.2f}m) 超出限制 (±{position_limit}m)')
                self.get_logger().warn(f'持续时间{self.get_clock().now().nanoseconds / 1e9 - self.start_time}s')
                self.stop_control_flag = True
                return

            # Compute control input u = -Kx
            u = -self.K @ self.x
            force = float(u[0])
            
            # Log control input periodically
            if abs(force - self.last_control) > 0.1 or self.control_count % 100 == 0:
                # self.get_logger().info('\n计算并发布控制指令')
                self.get_logger().info(f'\nCart position: {self.x.T[0][0]:.3f}, Cart velocity{self.x.T[0][1]:.3f}, ' + \
                                       f'Pole angle: {self.x.T[0][2]:.3f}, Pole angular velocity: {self.x.T[0][3]:.3f}, ' + \
                                       f'Control force: {force:.3f}N')
            
            # Publish control command
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
            
            self.last_control = force
            self.control_count += 1

            # 发布数据
            cart_pos_msg = Float64()
            cart_pos_msg.data = float(self.x[0][0])
            self.cart_pos_pub.publish(cart_pos_msg)
            
            cart_vel_msg = Float64()
            cart_vel_msg.data = float(self.x[1][0])
            self.cart_vel_pub.publish(cart_vel_msg)
            
            pole_angle_msg = Float64()
            pole_angle_msg.data = float(self.x[2][0])
            self.pole_angle_pub.publish(pole_angle_msg)
            
            pole_vel_msg = Float64()
            pole_vel_msg.data = float(self.x[3][0])
            self.pole_vel_pub.publish(pole_vel_msg)
            
            force_msg = Float64()
            force_msg.data = force
            self.control_force_pub.publish(force_msg)

            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 