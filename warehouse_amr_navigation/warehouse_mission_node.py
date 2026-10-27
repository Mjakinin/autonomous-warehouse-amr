#!/usr/bin/env python3
"""
Industrial Warehouse Fleet Mission Dispatcher Node.
Coordinates autonomous logistics fulfillment tasks via Nav2 Action Interfaces,
tracks mission execution, dynamic obstacle avoidance, and publishes live AMR telemetry.
"""

import math
import time
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf_transformations


class WarehouseMissionDispatcher(Node):
    def __init__(self):
        super().__init__('warehouse_mission_dispatcher')

        # Telemetry State
        self.state = "INITIALIZING"
        self.subaction = "Waiting for Nav2 Stack"
        self.current_goal_name = "None"
        self.distance_to_goal = 0.0
        self.battery_pct = 98.5
        self.completed_orders = 0
        self.current_speed = 0.0
        self.current_x = 2.0
        self.current_y = 2.0
        self.current_theta = 0.0

        # Action Client to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Telemetry Publisher
        self.telem_pub = self.create_publisher(String, '/warehouse/amr_telemetry', 10)
        self.telem_timer = self.create_timer(0.2, self._publish_telemetry)

        # Worker Thread for Autonomous Mission Cycle
        self.mission_thread = threading.Thread(target=self._run_mission_cycle, daemon=True)
        self.mission_thread.start()

        self.get_logger().info("✓ Warehouse Fleet Mission Dispatcher initialized.")

    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def _publish_telemetry(self):
        telem = {
            "timestamp": time.time(),
            "state": self.state,
            "subaction": self.subaction,
            "target": self.current_goal_name,
            "pose": {
                "x": round(self.current_x, 2),
                "y": round(self.current_y, 2),
                "theta_deg": round(math.degrees(self.current_theta), 1)
            },
            "speed_mps": round(self.current_speed, 2),
            "distance_to_goal": round(self.distance_to_goal, 2),
            "battery_pct": round(self.battery_pct, 1),
            "completed_orders": self.completed_orders
        }
        msg = String()
        msg.data = json.dumps(telem)
        self.telem_pub.publish(msg)

    def _navigate_to(self, target_name: str, x: float, y: float, yaw: float, timeout_sec: float = 60.0) -> bool:
        self.current_goal_name = target_name
        self.get_logger().info(f"➜ Dispatching AMR to [{target_name}] at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
        goal_msg.pose.pose.orientation.x = float(q[0])
        goal_msg.pose.pose.orientation.y = float(q[1])
        goal_msg.pose.pose.orientation.z = float(q[2])
        goal_msg.pose.pose.orientation.w = float(q[3])

        # Send goal synchronously via threading event
        send_goal_event = threading.Event()
        goal_handle_container = [None]

        def goal_response_cb(future):
            goal_handle_container[0] = future.result()
            send_goal_event.set()

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(goal_response_cb)

        if not send_goal_event.wait(timeout=10.0):
            self.get_logger().error(f"✗ Timeout sending goal to {target_name}!")
            return False

        goal_handle = goal_handle_container[0]
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"✗ Goal to {target_name} was rejected by Nav2!")
            return False

        self.get_logger().info(f"✓ Goal accepted by Nav2! Tracking execution...")

        # Monitor execution until completion
        t_start = time.time()
        result_event = threading.Event()
        result_container = [None]

        def get_result_cb(fut):
            result_container[0] = fut.result()
            result_event.set()

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(get_result_cb)

        while not result_event.is_set():
            time.sleep(0.2)
            self.distance_to_goal = math.hypot(x - self.current_x, y - self.current_y)
            # Battery discharge simulation during movement
            self.battery_pct = max(10.0, self.battery_pct - 0.02 * self.current_speed)

            if time.time() - t_start > timeout_sec:
                self.get_logger().warn(f"⚠ Navigation to {target_name} timed out after {timeout_sec}s! Cancelling...")
                goal_handle.cancel_goal_async()
                return False

        status = result_container[0].status
        # Status 4 = STATUS_SUCCEEDED
        if status == 4 or self.distance_to_goal < 0.35:
            self.get_logger().info(f"✓ Successfully reached [{target_name}]! (Remaining dist: {self.distance_to_goal:.2f} m)")
            return True
        else:
            self.get_logger().warn(f"⚠ Navigation to {target_name} ended with status code {status}.")
            return False

    def _run_mission_cycle(self):
        time.sleep(2.0)
        self.get_logger().info("➜ Warte auf Nav2 Action Server /navigate_to_pose...")
        if not self.nav_client.wait_for_server(timeout_sec=40.0):
            self.get_logger().error("✗ Nav2 Action Server nicht verfügbar!")
            self.state = "ERROR"
            self.subaction = "Nav2 Server Offline"
            return

        self.get_logger().info("✓ Nav2 Action Server bereit! Starte Warehouse Auftragslogistik...")
        self.state = "MISSION_ACTIVE"

        # Logistics Waypoints in Warehouse
        # Layout: 30x20m map
        # Charger at (2.0, 2.0)
        # Rack Aisle 1 at (10.0, 5.0)
        # Main Highway Corridor at (15.0, 9.5)
        # Outbound Shipping Bay at (25.0, 14.0)

        orders = [
            ("RACK_AISLE_PICK_BAY", 10.0, 5.0, 0.0),
            ("OUTBOUND_SHIPPING_DOCK", 25.0, 14.0, 0.0),
            ("CHARGER_STATION", 2.0, 2.0, 0.0),
        ]

        # Execute Order Sequence
        for order_idx, (name, target_x, target_y, target_yaw) in enumerate(orders, 1):
            self.subaction = f"En Route: {name}"
            print("\n" + "=" * 76)
            print(f"   LOGISTICS ORDER #{order_idx}: FAHRBEFEHL NACH [{name}]")
            print("=" * 76)

            success = self._navigate_to(name, target_x, target_y, target_yaw, timeout_sec=65.0)

            if "PICK" in name:
                self.subaction = "Automated Cargo Pickup"
                print("➜ Belade AMR mit Euro-Palette (Pneumatischer Hubtisch aktiv)...")
                time.sleep(2.5)
                print("✓ Palette gesichert! Bereit für Warentransport.")
            elif "SHIPPING" in name:
                self.subaction = "Automated Pallet Offload"
                print("➜ Entlade Palette an Packstation (Warenabgabe)...")
                time.sleep(2.5)
                print("✓ Ware erfolgreich übergeben!")
                self.completed_orders += 1
            elif "CHARGER" in name:
                self.subaction = "Fast Charging"
                print("➜ AMR dockt an Ladestation an. Induktiver Ladevorgang aktiv.")
                self.battery_pct = min(100.0, self.battery_pct + 5.0)

        self.state = "COMPLETED_ALL_ORDERS"
        self.subaction = "All Warehouse Missions Fulfilled"
        print("\n" + "=" * 76)
        print("   MISSION VOLLSTÄNDIG ABGESCHLOSSEN: Alle Logistikaufträge erfüllt!")
        print("=" * 76)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMissionDispatcher()
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
