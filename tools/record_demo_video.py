#!/usr/bin/env python3
"""
Automated Demo Video Recorder for Autonomous Warehouse AMR Navigation Stack.
Runs Nav2 & RViz2 in an offscreen Xvfb virtual frame buffer (:99),
records the complete intralogistics warehouse mission (Aisle navigation,
dynamic obstacle avoidance, cargo pickup & pallet offload) using FFmpeg,
and encodes both a modern web MP4 video and an animated GitHub README GIF.
"""

import os
import sys
import time
import json
import signal
import subprocess
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AMRRecorder(Node):
    def __init__(self):
        super().__init__('warehouse_amr_recorder')
        self.telemetry = None
        self.sub = self.create_subscription(
            String, '/warehouse/amr_telemetry', self._cb, 10
        )

    def _cb(self, msg: String):
        try:
            self.telemetry = json.loads(msg.data)
        except Exception:
            pass


def configure_amr_rviz():
    rviz_paths = [
        "/root/ros-projects/src/warehouse_amr_navigation/rviz/warehouse_amr.rviz",
        "/root/ros-projects/install/warehouse_amr_navigation/share/warehouse_amr_navigation/rviz/warehouse_amr.rviz",
    ]
    for p in rviz_paths:
        if os.path.exists(p):
            try:
                with open(p, "r") as f:
                    cfg = yaml.safe_load(f)

                # Set window geometry
                if "Window Geometry" in cfg:
                    cfg["Window Geometry"]["X"] = 0
                    cfg["Window Geometry"]["Y"] = 0
                    cfg["Window Geometry"]["Width"] = 1280
                    cfg["Window Geometry"]["Height"] = 720

                # Set camera view to center full 30x20m warehouse floor
                v = cfg.get("Visualization Manager", {}).get("Views", {}).get("Current", {})
                v["Class"] = "rviz_default_plugins/TopDownOrtho"
                v["X"] = 15.0
                v["Y"] = 10.0
                v["Scale"] = 28.0
                v["Angle"] = 0.0

                with open(p, "w") as f:
                    yaml.dump(cfg, f)
            except Exception as e:
                print(f"Warning: could not configure rviz: {e}")


def cleanup():
    try:
        my_pid = os.getpid()
        import re
        target = re.compile(r"warehouse_simulator|rviz2|amcl|planner_server|controller_server|behavior_server|bt_navigator|map_server|lifecycle_manager|component_container_isolated|warehouse_amr|Xvfb|ffmpeg")
        for pid_str in os.listdir("/proc"):
            if pid_str.isdigit():
                p = int(pid_str)
                if p not in (1, my_pid):
                    try:
                        with open(f"/proc/{p}/cmdline", "rb") as f:
                            c = f.read().decode("utf-8", "ignore")
                        if target.search(c) and "record_demo_video" not in c:
                            os.kill(p, signal.SIGKILL)
                    except Exception:
                        pass
    except Exception:
        pass
    subprocess.run("rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /tmp/.X99-lock /tmp/.X11-unix/X99", shell=True, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def main():
    print("=" * 78)
    print("   AUTONOMOUS WAREHOUSE AMR — AUTOMATED DEMO VIDEO RECORDER")
    print("=" * 78)

    cleanup()
    configure_amr_rviz()

    media_dir = "/root/ros-projects/src/warehouse_amr_navigation/docs/media"
    os.makedirs(media_dir, exist_ok=True)
    raw_mp4 = "/tmp/raw_warehouse_amr_demo.mp4"
    if os.path.exists(raw_mp4):
        os.remove(raw_mp4)

    # 1. Start Xvfb Virtual Framebuffer (:99 at 1280x720)
    print("➜ [1/5] Starte Xvfb Virtual Display (:99, 1280x720)...")
    xvfb_proc = subprocess.Popen(
        ["Xvfb", ":99", "-screen", "0", "1280x720x24", "-ac"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1.5)

    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    env["PYTHONUNBUFFERED"] = "1"

    # 2. Start FFmpeg screen capture on :99.0
    print("➜ [2/5] Initialisiere FFmpeg HD Video Recorder...")
    ffmpeg_cmd = [
        "ffmpeg", "-y",
        "-f", "x11grab",
        "-video_size", "1280x720",
        "-framerate", "25",
        "-draw_mouse", "0",
        "-i", ":99.0",
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-pix_fmt", "yuv420p",
        raw_mp4
    ]
    ffmpeg_proc = subprocess.Popen(
        ffmpeg_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1.0)

    # 3. Launch Nav2 & Warehouse AMR Stack with RViz2 & Autonomous Mission
    print("➜ [3/5] Starte Nav2 Stack, AMCL, Simulator, RViz2 & Mission Dispatcher...")
    launch_cmd = [
        "ros2", "launch", "warehouse_amr_navigation", "warehouse_amr.launch.py",
        "use_rviz:=true", "auto_mission:=true", "dynamic_obstacle:=true"
    ]
    launch_proc = subprocess.Popen(
        launch_cmd,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    rclpy.init()
    recorder_node = AMRRecorder()

    print("➜ [4/5] Zeichne autonome Warehouse-Mission auf (Telemetry Tracking)...")
    t0 = time.time()
    max_duration = 75.0
    last_subaction = ""

    try:
        while rclpy.ok() and (time.time() - t0 < max_duration):
            rclpy.spin_once(recorder_node, timeout_sec=0.2)
            if recorder_node.telemetry:
                st = recorder_node.telemetry.get("state", "")
                sub = recorder_node.telemetry.get("subaction", "")
                tgt = recorder_node.telemetry.get("target", "") or recorder_node.telemetry.get("target_waypoint", "")
                bat = recorder_node.telemetry.get("battery_pct", 100.0)

                if sub != last_subaction:
                    print(f"   [{time.time()-t0:4.1f}s] FSM: {st:<18} | Action: {sub:<30} | Battery: {bat:.1f}%")
                    last_subaction = sub

                # Mission completes when AMR returns to charging station
                if st == "COMPLETED_ALL_ORDERS" or ("CHARGER" in tgt and ("Completed" in sub or "Charging" in sub or "Docked" in sub or "Fulfilled" in sub)):
                    print("   ✓ AMR hat alle Logistik-Aufträge abgeschlossen und dockt an Ladestation an!")
                    time.sleep(3.0)
                    break
    finally:
        print("➜ [5/5] Beende Aufzeichnung & kodiere Videoformate...")
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=5)
        except Exception:
            pass

        try:
            ffmpeg_proc.send_signal(signal.SIGINT)
            ffmpeg_proc.wait(timeout=10)
        except Exception:
            ffmpeg_proc.kill()

        try:
            xvfb_proc.terminate()
            xvfb_proc.wait(timeout=3)
        except Exception:
            xvfb_proc.kill()

        recorder_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not os.path.exists(raw_mp4) or os.path.getsize(raw_mp4) < 10000:
        print("✗ Fehler: Raw-Videodatei wurde nicht erfolgreich erzeugt!")
        return 1

    final_mp4 = os.path.join(media_dir, "warehouse_amr_demo.mp4")
    final_gif = os.path.join(media_dir, "warehouse_amr_demo.gif")

    # Crop pure 2D warehouse map viewport (excluding left dock, top menu bar, and bottom status bar)
    crop_filter = "crop=860:608:405:82"

    # Encode optimized web-friendly MP4 (H.264, FastStart)
    print(f"➜ Optimiere MP4 Video -> {final_mp4}...")
    subprocess.run([
        "ffmpeg", "-y", "-i", raw_mp4,
        "-vf", f"{crop_filter},scale=860:608",
        "-c:v", "libx264", "-crf", "22", "-preset", "medium",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        final_mp4
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Encode lightweight animated GIF for GitHub README preview (10 fps, 600px)
    print(f"➜ Erzeuge GitHub README GIF -> {final_gif}...")
    subprocess.run([
        "ffmpeg", "-y", "-i", raw_mp4,
        "-vf", f"{crop_filter},fps=10,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
        "-loop", "0",
        final_gif
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("=" * 78)
    print(" ✓ VIDEOAUFZEICHNUNG ERFOLGREICH ABGESCHLOSSEN!")
    print(f"   • MP4: {final_mp4} ({os.path.getsize(final_mp4) / (1024*1024):.1f} MB)")
    print(f"   • GIF: {final_gif} ({os.path.getsize(final_gif) / (1024*1024):.1f} MB)")
    print("=" * 78)

    cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
