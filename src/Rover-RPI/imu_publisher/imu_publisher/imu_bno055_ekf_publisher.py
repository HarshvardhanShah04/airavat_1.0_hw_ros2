#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

import board
import busio
import adafruit_bno055


class BNO055IMUNode(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Changed from RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher
        self.pub = self.create_publisher(Imu, '/imu/data', qos_profile)

        # I2C + IMU
        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

        # IMU-only fusion (no magnetometer)
        self.imu.mode = adafruit_bno055.IMUPLUS_MODE

        # Publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_imu)

        self.get_logger().info(
            "BNO055 IMU node started (gyro + accel with covariances)"
        )

    def publish_imu(self):
        gyro = self.imu.gyro
        accel = self.imu.acceleration

        # Case 1: driver returned None
        if gyro is None or accel is None:
            return

        gx_raw, gy_raw, gz_raw = gyro
        ax_raw, ay_raw, az_raw = accel

        # Case 2: driver returned (None, None, None)
        if None in (gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw):
            return

        # REMAP RAW READINGS TO ROS CONVENTION (+X=forward, +Y=left, +Z=up)
        # Your IMU: accel +Y=forward, +X=right, +Z=down
        #           gyro axes inverted from accel
        
        # Accelerometer remapping:
        ax = -ay_raw   # IMU Y → Robot X (forward)
        ay = ax_raw  # IMU -X → Robot Y (left)
        az = -az_raw  # IMU -Z → Robot Z (up)
        
        # Gyroscope remapping:
        gx = gy_raw  # IMU -Y → Robot X (pitch rate)
        gy = -gx_raw   # IMU +X → Robot Y (roll rate)
        gz = gz_raw  # IMU -Z → Robot Z (yaw rate)

        self.get_logger().info(f"g: {gx}, {gy}, {gz} | a: {ax}, {ay}, {az}")

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Disable orientation (tell EKF not to use it)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        # Angular velocity (rad/s) - now using remapped values
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        
        # GYRO COVARIANCE (3x3 matrix = 9 values, row-major)
        # BNO055 gyro noise: ~0.014 rad/s → variance ≈ 0.0002 rad²/s²
        # Using conservative 0.02 for real-world conditions
        msg.angular_velocity_covariance = [
            0.02, 0.0,  0.0,
            0.0,  0.02, 0.0,
            0.0,  0.0,  0.02
        ]

        # Linear acceleration (m/s²) - now using remapped values
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        
        #ACCEL COVARIANCE (3x3 matrix = 9 values, row-major)
        # BNO055 accel noise: ~0.2 m/s² → variance ≈ 0.04 m²/s⁴
        msg.linear_acceleration_covariance = [
            3.0, 0.0,  0.0,
            0.0,  3.0, 0.0,
            0.0,  0.0,  3.0
        ]

        self.pub.publish(msg)



def main():
    rclpy.init()
    node = BNO055IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()