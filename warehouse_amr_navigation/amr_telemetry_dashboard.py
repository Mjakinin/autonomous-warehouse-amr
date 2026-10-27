#!/usr/bin/env python3
"""
Live ANSI Terminal Telemetry Dashboard for Warehouse AMR Navigation Stack.
"""

import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AMRTelemetryDashboard(Node):
    def __init__(self):
        super().__init__('amr_telemetry_dashboard')
        self.sub = self.create_subscription(String, '/warehouse/amr_telemetry', self._cb, 10)
        self.last_state = ""

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "UNKNOWN")
            subaction = data.get("subaction", "")
            target = data.get("target", "")
            p = data.get("pose", {})
            x = p.get("x", 0.0)
            y = p.get("y", 0.0)
            th = p.get("theta_deg", 0.0)
            spd = data.get("speed_mps", 0.0)
            dist = data.get("distance_to_goal", 0.0)
            bat = data.get("battery_pct", 100.0)
            orders = data.get("completed_orders", 0)

            # Draw progress bar for speed
            spd_bar_len = min(20, int((spd / 0.65) * 20))
            spd_bar = "█" * spd_bar_len + "░" * (20 - spd_bar_len)

            # Battery icon
            bat_icon = "🔋" if bat > 20 else "🪫"

            line = f"\r[{state:<18}] Pose: ({x:5.2f}, {y:5.2f}, {th:5.1f}°) | Spd: {spd:4.2f} m/s [{spd_bar}] | Dist: {dist:5.2f}m | {bat_icon} {bat:4.1f}% | Orders: {orders}"
            sys.stdout.write(line)
            sys.stdout.flush()

            if state != self.last_state:
                sys.stdout.write(f"\n➜ [MISSION UPDATE] State changed to: {state} ({subaction})\n")
                sys.stdout.flush()
                self.last_state = state
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AMRTelemetryDashboard()
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
