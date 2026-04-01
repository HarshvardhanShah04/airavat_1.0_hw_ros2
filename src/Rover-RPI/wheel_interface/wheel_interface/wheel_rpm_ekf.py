#!/usr/bin/env python3
import math
import serial
import threading
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


TICKS_PER_REV = 1078.0
WHEEL_RADIUS = 0.035      # m
WHEEL_BASE = 0.25         # m
MAX_MOTOR_RPM = 150

BAUDRATE = 115200


class WheelInterfaceNode(Node):
    def __init__(self):
        super().__init__("wheel_interface_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.port = self.get_parameter("port").get_parameter_value().string_value

        # Serial setup with 50ms timeout (odometry arrives at 50Hz = 20ms)
        self.ser = serial.Serial(self.port, BAUDRATE, timeout=0.05)
        self.get_logger().info(f"Opened serial port {self.port} @ {BAUDRATE}")

        if not self.ser.is_open:
            self.get_logger().error("Failed to open the serial port")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", qos_profile)
        
        # Note: TF odom->base_footprint is handled by robot_localization EKF
        # This node only publishes wheel odometry data

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.joint_names = [
            "ch_wheel_FL",
            "ch_wheel_FR",
            "ch_wheel_RL",
            "ch_wheel_RR",
        ]
        self.num_wheels = len(self.joint_names)


        self.state_lock = threading.Lock()

        self.positions = [0.0] * self.num_wheels   # θ (rad), unbounded
        self.velocities = [0.0] * self.num_wheels  # ω (rad/s)

        # Robot pose in odom frame (handled by wheel odometry, EKF will fuse)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Thread control
        self._running = True
        self.serial_thread = threading.Thread(
            target=self.serial_loop, daemon=True
        )
        self.serial_thread.start()

        self.get_logger().info("Wheel Interface Node Initialized (Nav2 compatible)")

    # CMD_VEL → RPM → SERIAL
    def cmd_vel_callback(self, msg: Twist):
        """
        Receive Twist from Nav2, convert to RPM for each wheel,
        and send via serial to ESP32.
        """
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port closed, cannot send motor command")
            return

        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics
        v_left = v - (w * WHEEL_BASE / 2.0)
        v_right = v + (w * WHEEL_BASE / 2.0)

        # Convert linear velocity to RPM
        rpm_left = self.linear_to_rpm(v_left)
        rpm_right = self.linear_to_rpm(v_right)

        # RPM commands: FL, FR, RL, RR
        rpm_cmds = [
            rpm_left,   # FL
            rpm_right,  # FR
            rpm_left,   # RL
            rpm_right   # RR
        ]

        vals = [
            max(-MAX_MOTOR_RPM, min(MAX_MOTOR_RPM, int(r)))
            for r in rpm_cmds
        ]

        payload = struct.pack('<hhhh', *vals)

        checksum = 0
        for b in payload:
            checksum ^= b

        packet = payload + struct.pack('<B', checksum)

        try:
            self.ser.write(packet)
            self.get_logger().info(f"Sent RPM: FL={vals[0]}, FR={vals[1]}, RL={vals[2]}, RR={vals[3]}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def linear_to_rpm(self, v):
        """Convert linear velocity [m/s] to wheel RPM."""
        wheel_rpm = (v / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0
        return wheel_rpm

    # SERIAL RX → ODOM
    def serial_loop(self):
        """Block on serial, process encoder packets whenever they arrive."""
        while rclpy.ok() and self._running:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                continue

            if not line:
                continue

            # Expect format: "DATA:dC0,dC1,dC2,dC3,dt_micro"
            if line.startswith("DATA:"):
                payload = line[len("DATA:"):]
                try:
                    parts = [int(p.strip()) for p in payload.split(",")]
                except ValueError:
                    self.get_logger().warn(f"Non-integer payload: {payload}")
                    continue

                self.process_packet(parts)

    def process_packet(self, parts):
        """Process encoder data: [dC0, dC1, dC2, dC3, dt_micro]."""
        if len(parts) != 5:
            self.get_logger().warn(f"Bad packet (field count): {len(parts)}")
            return

        delta_counts = parts[0:4]
        dt = parts[4] / 1000000.0  # Convert microseconds to seconds

        if dt <= 0 or dt > 1.0:  # Sanity check: dt should be ~20ms for 50Hz
            self.get_logger().warn(f"Invalid dt: {dt}")
            return
        
        # Safety check for overflow/corrupt data
        if any(abs(c) > 100000 for c in delta_counts):
            self.get_logger().warn(f"Suspicious encoder counts: {delta_counts}")
            return

        # Thread-safe state update
        with self.state_lock:
            for i in range(self.num_wheels):
                # Δθ in radians for this wheel
                dtheta = (delta_counts[i] / TICKS_PER_REV) * (2.0 * math.pi)

                # Update cumulative position and velocity
                self.positions[i] += dtheta
                self.velocities[i] = dtheta / dt

            omega_left = 0.5 * (self.velocities[0] + self.velocities[2])
            omega_right = 0.5 * (self.velocities[1] + self.velocities[3])

            v_left = omega_left * WHEEL_RADIUS
            v_right = omega_right * WHEEL_RADIUS

            v = 0.5 * (v_right + v_left)              # [m/s]
            omega = (v_right - v_left) / WHEEL_BASE   # [rad/s]
            
            if not math.isfinite(v) or not math.isfinite(omega):
                self.get_logger().warn("Non-finite velocity calculated, skipping update")
                return

            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += omega * dt

            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            positions_copy = self.positions.copy()
            velocities_copy = self.velocities.copy()
            x, y, yaw = self.x, self.y, self.yaw

        now = self.get_clock().now()
        
        self.get_logger().info(
            f"Odom: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}° | v={v:.2f}m/s"
        )

        # --- Publish Joint States ---
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = positions_copy
        js.velocity = velocities_copy
        self.joint_pub.publish(js)

        # --- Publish Odometry ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        half_yaw = yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # Covariance matrices (EKF will use these)
        odom.pose.covariance = [
            0.02, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.02, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]

        odom.twist.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1
        ]

        self.odom_pub.publish(odom)


    # SHUTDOWN
    def destroy_node(self):
        """Clean shutdown: stop motors, close serial, join thread."""
        self._running = False

        # Send stop command
        try:
            if self.ser and self.ser.is_open:
                payload = struct.pack('<hhhh', 0, 0, 0, 0)
                checksum = 0
                for b in payload:
                    checksum ^= b
                packet = payload + struct.pack('<B', checksum)
                self.ser.write(packet)
                self.get_logger().info("Sent stop command (0 RPM)")
                self.ser.close()
        except Exception as e:
            self.get_logger().warn(f"Error during shutdown: {e}")

        # Wait for thread to exit
        try:
            if self.serial_thread.is_alive():
                self.serial_thread.join(timeout=0.5)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down wheel_interface_node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()