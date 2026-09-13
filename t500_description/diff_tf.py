#!/usr/bin/env python3
"""
Encoder-based odometry node for the real T500 robot.
Fuses wheel encoder ticks (/lwheel, /rwheel) with /cmd_vel and publishes
odom -> base_footprint, replacing Gazebo's diff_drive plugin on hardware.
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster
import tf_transformations
from collections import deque

class DiffTfPrecise(Node):
    def __init__(self):
        super().__init__('diff_tf_precise')

        # === ROBOT PARAMETERS (must match urdf/t500.xacro geometry) ===
        self.WHEEL_RADIUS = 0.0635         # meters (wheel_diameter 0.127 / 2)
        self.WHEEL_SEPARATION = 0.6376     # meters (left/right_wheel_joint origins)
        self.TICKS_PER_REV = 102           # encoder ticks per revolution -- verify against your hardware

        # Encoder limits
        self.ENCODER_MIN = -32768
        self.ENCODER_MAX = 32767

        # === SENSOR FUSION PARAMETERS ===
        self.USE_ENCODER_FUSION = True
        self.ENCODER_TRUST = 0.8           # How much to trust encoders (0.0-1.0)
        self.CMD_VEL_TRUST = 0.2           # How much to trust cmd_vel

        # Adaptive trust based on motion type
        self.STRAIGHT_TRUST = 0.9          # Trust encoders more in straight motion
        self.TURN_TRUST = 0.7              # Trust encoders less in turns

        # === CALIBRATION VARIABLES ===
        self.calibration_mode = False
        self.calibration_samples = []
        self.calibration_distance = 0.0

        # === STATE VARIABLES ===
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        # Encoder data
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left = 0
        self.prev_right = 0
        self.initial_left = 0
        self.initial_right = 0

        # Commanded velocities
        self.cmd_vx = 0.0
        self.cmd_vth = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Time tracking
        self.last_update_time = self.get_clock().now()
        self.last_enc_time = self.get_clock().now()

        # History for smoothing
        self.velocity_history = deque(maxlen=10)
        self.angular_history = deque(maxlen=10)

        # Statistics
        self.total_distance = 0.0
        self.total_rotation = 0.0
        self.update_count = 0

        # Conversion factor
        self.METERS_PER_TICK = (2.0 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV

        # === SETUP ROS2 ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.left_sub = self.create_subscription(
            Int32, '/lwheel', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Int32, '/rwheel', self.right_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Services for calibration
        from std_srvs.srv import Empty
        self.reset_srv = self.create_service(
            Empty, 'reset_odometry', self.reset_odometry_callback)
        self.calibrate_srv = self.create_service(
            Empty, 'start_calibration', self.start_calibration_callback)

        # Timers
        self.odom_timer = self.create_timer(0.02, self.update_odometry)  # 50Hz
        self.status_timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info("Precise odometry node started")
        self.get_logger().info(f"Wheel radius: {self.WHEEL_RADIUS}m")
        self.get_logger().info(f"Wheel separation: {self.WHEEL_SEPARATION}m")
        self.get_logger().info(f"Sensor fusion: {self.USE_ENCODER_FUSION}")

        # Reset initial encoder values
        self.initial_left = self.left_ticks
        self.initial_right = self.right_ticks

    def left_callback(self, msg):
        self.left_ticks = msg.data
        self.last_enc_time = self.get_clock().now()

    def right_callback(self, msg):
        self.right_ticks = msg.data
        self.last_enc_time = self.get_clock().now()

    def cmd_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vth = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def handle_encoder_overflow(self, diff):
        """Handle encoder overflow when ticks wrap around"""
        max_diff = (self.ENCODER_MAX - self.ENCODER_MIN) // 2

        if diff > max_diff:
            return diff - (self.ENCODER_MAX - self.ENCODER_MIN + 1)
        elif diff < -max_diff:
            return diff + (self.ENCODER_MAX - self.ENCODER_MIN + 1)
        return diff

    def calculate_wheel_velocities(self, dt):
        """Calculate wheel velocities from encoder ticks"""
        if dt <= 0:
            return 0.0, 0.0

        # Get encoder differences
        left_diff = self.handle_encoder_overflow(self.left_ticks - self.prev_left)
        right_diff = self.handle_encoder_overflow(self.right_ticks - self.prev_right)

        # Convert ticks to meters
        left_dist = left_diff * self.METERS_PER_TICK
        right_dist = right_diff * self.METERS_PER_TICK

        # Calculate velocities (m/s)
        left_vel = left_dist / dt
        right_vel = right_dist / dt

        # Save current ticks
        self.prev_left = self.left_ticks
        self.prev_right = self.right_ticks

        return left_vel, right_vel

    def calculate_robot_velocity(self, left_vel, right_vel):
        """Convert wheel velocities to robot velocity"""
        # Differential drive kinematics
        vx = (right_vel + left_vel) / 2.0
        vth = (right_vel - left_vel) / self.WHEEL_SEPARATION
        return vx, vth

    def adaptive_sensor_fusion(self, enc_vx, enc_vth, dt):
        """Intelligently fuse encoder and cmd_vel data"""
        # Calculate time since last command
        cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if not self.USE_ENCODER_FUSION or cmd_age > 1.0:
            # No recent cmd_vel, use encoders only
            return enc_vx, enc_vth

        # Determine motion type for adaptive trust
        is_straight = abs(self.cmd_vth) < 0.1  # Small angular velocity
        is_turning = abs(self.cmd_vx) < 0.1 and abs(self.cmd_vth) > 0.1

        if is_straight:
            # Trust encoders more in straight motion
            encoder_trust = self.STRAIGHT_TRUST
        elif is_turning:
            # Trust encoders less in turning (encoders can be less accurate)
            encoder_trust = self.TURN_TRUST
        else:
            encoder_trust = self.ENCODER_TRUST

        cmd_trust = 1.0 - encoder_trust

        # Fuse velocities
        fused_vx = (encoder_trust * enc_vx) + (cmd_trust * self.cmd_vx)
        fused_vth = (encoder_trust * enc_vth) + (cmd_trust * self.cmd_vth)

        # Apply constraints
        max_vx = 1.0  # m/s
        max_vth = 3.0  # rad/s
        fused_vx = max(-max_vx, min(max_vx, fused_vx))
        fused_vth = max(-max_vth, min(max_vth, fused_vth))

        return fused_vx, fused_vth

    def update_odometry(self):
        """Main odometry update with sensor fusion"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # 1. Calculate velocity from encoders
        left_vel, right_vel = self.calculate_wheel_velocities(dt)
        enc_vx, enc_vth = self.calculate_robot_velocity(left_vel, right_vel)

        # 2. Fuse with cmd_vel using adaptive trust
        fused_vx, fused_vth = self.adaptive_sensor_fusion(enc_vx, enc_vth, dt)

        # Apply smoothing using moving average
        self.velocity_history.append(fused_vx)
        self.angular_history.append(fused_vth)

        smoothed_vx = np.mean(self.velocity_history) if self.velocity_history else fused_vx
        smoothed_vth = np.mean(self.angular_history) if self.angular_history else fused_vth

        # Update velocities
        self.vx = smoothed_vx
        self.vth = smoothed_vth

        # 3. Update position (using trapezoidal integration for accuracy)
        if abs(self.vx) > 0.001 or abs(self.vth) > 0.001:
            # Calculate displacement
            delta_x = self.vx * math.cos(self.theta) * dt
            delta_y = self.vx * math.sin(self.theta) * dt
            delta_theta = self.vth * dt

            # Update pose
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Update statistics
            self.total_distance += math.sqrt(delta_x**2 + delta_y**2)
            self.total_rotation += abs(delta_theta)

            # Normalize angle
            if self.theta > math.pi:
                self.theta -= 2 * math.pi
            elif self.theta < -math.pi:
                self.theta += 2 * math.pi

        # 4. Publish odometry
        self.publish_odometry(current_time)

        # 5. Update time
        self.last_update_time = current_time
        self.update_count += 1

        # 6. Calibration mode (if active)
        if self.calibration_mode:
            self.calibration_samples.append({
                'time': current_time,
                'left': self.left_ticks,
                'right': self.right_ticks,
                'cmd_vx': self.cmd_vx,
                'cmd_vth': self.cmd_vth
            })

    def publish_odometry(self, time):
        """Publish odometry message and TF transform"""
        # Create quaternion from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.vth

        # Calculate covariance based on motion type
        if abs(self.vx) > 0.1:  # Moving fast
            pose_cov = 0.02
            twist_cov = 0.01
        elif abs(self.vth) > 0.5:  # Turning fast
            pose_cov = 0.05
            twist_cov = 0.02
        else:  # Slow or stopped
            pose_cov = 0.005
            twist_cov = 0.002

        # Set covariance matrices
        odom_msg.pose.covariance = [pose_cov] * 36
        odom_msg.twist.covariance = [twist_cov] * 36

        self.odom_pub.publish(odom_msg)

        # TF transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    def reset_odometry_callback(self, request, response):
        """Reset odometry to zero"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.prev_left = self.left_ticks
        self.prev_right = self.right_ticks
        self.initial_left = self.left_ticks
        self.initial_right = self.right_ticks
        self.total_distance = 0.0
        self.total_rotation = 0.0

        self.get_logger().info("Odometry reset to zero")
        return response

    def start_calibration_callback(self, request, response):
        """Start calibration mode"""
        self.calibration_mode = True
        self.calibration_samples = []
        self.calibration_distance = 0.0

        self.get_logger().info("Calibration mode started")
        self.get_logger().info("Move robot forward 1 meter for calibration")
        return response

    def print_status(self):
        """Print status information"""
        # Calculate encoder-based distance
        left_diff = self.handle_encoder_overflow(self.left_ticks - self.initial_left)
        right_diff = self.handle_encoder_overflow(self.right_ticks - self.initial_right)

        left_dist = abs(left_diff * self.METERS_PER_TICK)
        right_dist = abs(right_diff * self.METERS_PER_TICK)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Position: x={self.x:.3f}m, y={self.y:.3f}m, theta={self.theta:.3f}rad")
        self.get_logger().info(f"Velocity: vx={self.vx:.3f}m/s, vth={self.vth:.3f}rad/s")
        self.get_logger().info(f"Encoder ticks: L={self.left_ticks}, R={self.right_ticks}")
        self.get_logger().info(f"Encoder distance: L={left_dist:.3f}m, R={right_dist:.3f}m")
        self.get_logger().info(f"Total traveled: {self.total_distance:.2f}m, rotated: {self.total_rotation:.2f}rad")
        self.get_logger().info(f"Updates: {self.update_count}")
        self.get_logger().info("=" * 60)

        # Check for encoder direction mismatch
        if left_diff * right_diff < 0:  # Different signs
            self.get_logger().warn("Encoders moving in opposite directions!")
            self.get_logger().warn("   Left diff: {}, Right diff: {}".format(left_diff, right_diff))

        # Check for excessive slip
        if abs(left_dist - right_dist) > 0.1 * max(left_dist, right_dist):
            self.get_logger().warn("Significant wheel slip detected!")

def main(args=None):
    rclpy.init(args=args)
    node = DiffTfPrecise()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
