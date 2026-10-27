#!/usr/bin/env python3
"""
High-Fidelity Deterministic Warehouse AMR Simulator Node.
Simulates 50 Hz differential-drive kinematics, 2D safety LiDAR raycasting against
the warehouse occupancy grid map, dynamic obstacle injection, and TF broadcasting.
"""

import os
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf_transformations
from ament_index_python.packages import get_package_share_directory


class WarehouseSimulatorNode(Node):
    def __init__(self):
        super().__init__('warehouse_simulator_node')

        # Parameters
        self.declare_parameter('init_x', 2.0)
        self.declare_parameter('init_y', 2.0)
        self.declare_parameter('init_theta', 0.0)
        self.declare_parameter('max_lin_vel', 0.65)
        self.declare_parameter('max_ang_vel', 1.0)
        self.declare_parameter('max_lin_acc', 1.5)
        self.declare_parameter('max_ang_acc', 2.5)
        self.declare_parameter('lidar_rate', 12.0)
        self.declare_parameter('enable_dynamic_obstacle', True)

        self.x = self.get_parameter('init_x').get_parameter_value().double_value
        self.y = self.get_parameter('init_y').get_parameter_value().double_value
        self.theta = self.get_parameter('init_theta').get_parameter_value().double_value

        self.v = 0.0
        self.w = 0.0
        self.target_v = 0.0
        self.target_w = 0.0

        self.max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter('max_ang_vel').get_parameter_value().double_value
        self.max_lin_acc = self.get_parameter('max_lin_acc').get_parameter_value().double_value
        self.max_ang_acc = self.get_parameter('max_ang_acc').get_parameter_value().double_value
        self.enable_dynamic_obstacle = self.get_parameter('enable_dynamic_obstacle').get_parameter_value().bool_value

        # Dynamic obstacle parameters (Autonomous Forklift / Worker Crossing)
        self.obs_x = 10.0
        self.obs_y = 7.5
        self.obs_vy = 0.45
        self.obs_radius = 0.40

        # Load Map for Raycasting
        self._load_warehouse_map()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/warehouse/amr_marker', 10)
        self.obs_marker_pub = self.create_publisher(MarkerArray, '/warehouse/dynamic_obstacles', 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._broadcast_static_transforms()

        # Timers
        self.dt_kinematics = 0.02  # 50 Hz
        self.kinematics_timer = self.create_timer(self.dt_kinematics, self._kinematics_loop)

        lidar_period = 1.0 / self.get_parameter('lidar_rate').get_parameter_value().double_value
        self.lidar_timer = self.create_timer(lidar_period, self._lidar_loop)
        self.marker_timer = self.create_timer(0.2, self._publish_markers)

        self.last_kinematics_time = time.time()
        self.get_logger().info(f"✓ Warehouse AMR Simulator initialized at ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.1f}°)")

    def _load_warehouse_map(self):
        try:
            pkg_share = get_package_share_directory('warehouse_amr_navigation')
            map_path = os.path.join(pkg_share, 'maps', 'warehouse_map.pgm')
        except Exception:
            map_path = '/home/mjakinin/ros-projects/src/warehouse_amr_navigation/maps/warehouse_map.pgm'

        self.map_res = 0.05
        with open(map_path, 'rb') as f:
            header1 = f.readline().decode()  # P5
            dims = f.readline().decode()
            while dims.startswith('#'):
                dims = f.readline().decode()
            w_str, h_str = dims.strip().split()
            self.map_w = int(w_str)
            self.map_h = int(h_str)
            max_val = int(f.readline().decode().strip())
            raw_bytes = f.read()

        # Reshape and flip Y so index (0,0) is bottom-left (ROS map origin)
        img = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((self.map_h, self.map_w))
        self.grid = np.flipud(img)
        # 0 in PGM is occupied (black walls/racks); 254 is free space
        self.occupied_mask = (self.grid < 128)
        self.get_logger().info(f"Loaded warehouse map: {self.map_w}x{self.map_h} cells, res={self.map_res}m")

    def _broadcast_static_transforms(self):
        # base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'base_footprint'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.05
        t_base.transform.rotation.w = 1.0

        # base_link -> laser_link (Mounted at front of AMR)
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser_link'
        t_laser.transform.translation.x = 0.28
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.22
        t_laser.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([t_base, t_laser])

    def _cmd_vel_cb(self, msg: Twist):
        self.target_v = float(np.clip(msg.linear.x, -self.max_lin_vel, self.max_lin_vel))
        self.target_w = float(np.clip(msg.angular.z, -self.max_ang_vel, self.max_ang_vel))

    def _kinematics_loop(self):
        now = time.time()
        dt = min(now - self.last_kinematics_time, 0.05)
        self.last_kinematics_time = now

        # Velocity ramp with acceleration limits
        dv = self.target_v - self.v
        max_dv = self.max_lin_acc * dt
        self.v += float(np.clip(dv, -max_dv, max_dv))

        dw = self.target_w - self.w
        max_dw = self.max_ang_acc * dt
        self.w += float(np.clip(dw, -max_dw, max_dw))

        # Integrate pose
        self.theta += self.w * dt
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # Update dynamic obstacle position (Patrolling across aisle)
        if self.enable_dynamic_obstacle:
            self.obs_y += self.obs_vy * dt
            if self.obs_y > 11.5:
                self.obs_y = 11.5
                self.obs_vy = -abs(self.obs_vy)
            elif self.obs_y < 7.5:
                self.obs_y = 7.5
                self.obs_vy = abs(self.obs_vy)

        stamp = self.get_clock().now().to_msg()
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)

        # Broadcast odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # Publish IMU
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity.z = self.w
        imu.linear_acceleration.x = dv / max(dt, 1e-4)
        self.imu_pub.publish(imu)

    def _lidar_loop(self):
        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        n_rays = 240
        scan.angle_increment = 2.0 * math.pi / n_rays
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 12.0
        scan.range_min = 0.10
        scan.range_max = 12.0

        # Laser position in world
        laser_offset = 0.28
        lx = self.x + laser_offset * math.cos(self.theta)
        ly = self.y + laser_offset * math.sin(self.theta)

        angles = np.linspace(-np.pi, np.pi, n_rays, endpoint=False) + self.theta
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)

        step = 0.04
        radii = np.arange(1, int(scan.range_max / step) + 1) * step

        # Vectorized ray marching against static warehouse map
        xs = (lx + np.outer(cos_a, radii)) / self.map_res
        ys = (ly + np.outer(sin_a, radii)) / self.map_res

        px = np.clip(xs.astype(np.int32), 0, self.map_w - 1)
        py = np.clip(ys.astype(np.int32), 0, self.map_h - 1)

        hits = self.occupied_mask[py, px]
        first_hits = np.argmax(hits, axis=1)
        has_hits = np.any(hits, axis=1)
        ranges = np.where(has_hits, radii[first_hits], scan.range_max)

        # Check intersection with dynamic obstacle (Worker / Forklift)
        if self.enable_dynamic_obstacle:
            # Ray circle intersection: d = || (lx, ly) + r * (cos, sin) - (ox, oy) || <= R
            dx = self.obs_x - lx
            dy = self.obs_y - ly
            # Projection of obstacle center onto ray
            proj = dx * cos_a + dy * sin_a
            dist_sq = dx**2 + dy**2
            perp_dist_sq = dist_sq - proj**2
            in_cone = (proj > 0) & (perp_dist_sq >= 0) & (perp_dist_sq <= self.obs_radius**2)
            d_hit = proj - np.sqrt(np.maximum(0.0, self.obs_radius**2 - perp_dist_sq))
            ranges = np.where(in_cone & (d_hit < ranges), np.maximum(scan.range_min, d_hit), ranges)

        # Add Gaussian noise
        noise = np.random.normal(0.0, 0.005, size=n_rays)
        ranges = np.clip(ranges + noise, scan.range_min, scan.range_max)
        scan.ranges = [float(r) for r in ranges]

        self.scan_pub.publish(scan)

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # AMR Chassis (Safety Yellow Industrial AGV)
        m_body = Marker()
        m_body.header.stamp = stamp
        m_body.header.frame_id = 'base_link'
        m_body.ns = 'amr_body'
        m_body.id = 0
        m_body.type = Marker.CUBE
        m_body.action = Marker.ADD
        m_body.scale.x = 0.80
        m_body.scale.y = 0.58
        m_body.scale.z = 0.28
        m_body.pose.position.z = 0.14
        m_body.pose.orientation.w = 1.0
        m_body.color.r = 0.95
        m_body.color.g = 0.75
        m_body.color.b = 0.05
        m_body.color.a = 0.95
        ma.markers.append(m_body)

        # LiDAR Puck (Matte Black)
        m_lidar = Marker()
        m_lidar.header.stamp = stamp
        m_lidar.header.frame_id = 'laser_link'
        m_lidar.ns = 'amr_lidar'
        m_lidar.id = 1
        m_lidar.type = Marker.CYLINDER
        m_lidar.action = Marker.ADD
        m_lidar.scale.x = 0.12
        m_lidar.scale.y = 0.12
        m_lidar.scale.z = 0.08
        m_lidar.pose.orientation.w = 1.0
        m_lidar.color.r = 0.15
        m_lidar.color.g = 0.15
        m_lidar.color.b = 0.15
        m_lidar.color.a = 1.0
        ma.markers.append(m_lidar)

        # Direction Arrow
        m_arrow = Marker()
        m_arrow.header.stamp = stamp
        m_arrow.header.frame_id = 'base_link'
        m_arrow.ns = 'amr_heading'
        m_arrow.id = 2
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.scale.x = 0.45
        m_arrow.scale.y = 0.08
        m_arrow.scale.z = 0.08
        m_arrow.pose.position.x = 0.10
        m_arrow.pose.position.z = 0.30
        m_arrow.pose.orientation.w = 1.0
        m_arrow.color.r = 0.1
        m_arrow.color.g = 0.8
        m_arrow.color.b = 0.2
        m_arrow.color.a = 0.9
        ma.markers.append(m_arrow)

        self.marker_pub.publish(ma)

        # Dynamic Obstacle Marker (Orange Warehouse Worker / Forklift)
        if self.enable_dynamic_obstacle:
            oma = MarkerArray()
            m_obs = Marker()
            m_obs.header.stamp = stamp
            m_obs.header.frame_id = 'map'
            m_obs.ns = 'dynamic_obstacle'
            m_obs.id = 10
            m_obs.type = Marker.CYLINDER
            m_obs.action = Marker.ADD
            m_obs.pose.position.x = self.obs_x
            m_obs.pose.position.y = self.obs_y
            m_obs.pose.position.z = 0.75
            m_obs.pose.orientation.w = 1.0
            m_obs.scale.x = self.obs_radius * 2.0
            m_obs.scale.y = self.obs_radius * 2.0
            m_obs.scale.z = 1.50
            m_obs.color.r = 0.9
            m_obs.color.g = 0.2
            m_obs.color.b = 0.1
            m_obs.color.a = 0.95
            oma.markers.append(m_obs)

            # Warning text above obstacle
            m_text = Marker()
            m_text.header.stamp = stamp
            m_text.header.frame_id = 'map'
            m_text.ns = 'dynamic_obstacle'
            m_text.id = 11
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = self.obs_x
            m_text.pose.position.y = self.obs_y
            m_text.pose.position.z = 1.70
            m_text.scale.z = 0.35
            m_text.text = "DYNAMIC OBSTACLE [CROSSING]"
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 1.0
            oma.markers.append(m_text)

            self.obs_marker_pub.publish(oma)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseSimulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
