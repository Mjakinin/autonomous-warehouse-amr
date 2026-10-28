#!/usr/bin/env bash
# ==============================================================================
# Autonomous Warehouse AMR Navigation Stack — Master Launcher
# 2D-LiDAR SLAM, Costmaps, Nav2 & Intralogistics Mission Dispatcher
# ==============================================================================

# Allow Docker container to access host X11 display (for RViz2)
xhost +local:docker > /dev/null 2>&1 || true

# Ensure ros_container is running
if [ -z "$(docker ps -q -f name=ros_container)" ]; then
    echo "➜ Starte ros_container Docker-Container..."
    docker start ros_container > /dev/null 2>&1 || true
    sleep 1
fi

cleanup_amr() {
    echo "➜ Bereinige alte Prozesse & FastDDS Shared Memory..."
    docker exec ros_container python3 -c '
import os, signal, re
my_pid = os.getpid()
parent_pid = os.getppid()
ignore = {1, my_pid, parent_pid}
target = re.compile(r"warehouse_simulator|rviz2|amcl|planner_server|controller_server|behavior_server|bt_navigator|map_server|lifecycle_manager")
for pid_str in os.listdir("/proc"):
    if pid_str.isdigit():
        pid = int(pid_str)
        if pid in ignore:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().decode("utf-8", "ignore")
            if target.search(cmd):
                os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
' > /dev/null 2>&1 || true
    docker exec ros_container rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* > /dev/null 2>&1 || true
    sleep 0.5
}

# TTY detection for docker exec
DOCKER_IT="-i"
if [ -t 0 ]; then
    DOCKER_IT="-it"
fi

echo "=============================================================================="
echo "    AUTONOMOUS WAREHOUSE AMR (AGV) — INTRA-LOGISTICS NAVIGATION STACK"
echo "=============================================================================="
echo " Bitte wähle eine Option:"
echo " [1] Vollautomatischer Flotten-Auftrag + RViz2 GUI (Empfohlen)"
echo " [2] Manuelle Navigation (Ziel per 2D Nav Goal in RViz2 vorgeben)"
echo " [3] Automatisierter Regressionstest & Benchmark (Headless)"
echo " [4] Beenden"
echo "=============================================================================="
read -p "Auswahl [1-4] (Standard: 1): " CHOICE
CHOICE="${CHOICE:-1}"

if [ "$CHOICE" == "1" ]; then
    cleanup_amr
    echo "➜ Starte autonome Warehouse-Mission mit RViz2 Visualization & Live-Fahrt..."
    docker exec $DOCKER_IT ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      ros2 launch warehouse_amr_navigation warehouse_amr.launch.py use_rviz:=true auto_mission:=true"

elif [ "$CHOICE" == "2" ]; then
    cleanup_amr
    echo "➜ Starte Nav2 Bringup mit RViz2 (Manuelle Zielvorgabe)..."
    docker exec $DOCKER_IT ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      ros2 launch warehouse_amr_navigation warehouse_amr.launch.py use_rviz:=true auto_mission:=false"

elif [ "$CHOICE" == "3" ]; then
    cleanup_amr
    echo "➜ Führe automatisierten Nav2 Regressionstest aus..."
    docker exec $DOCKER_IT ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      python3 /root/ros-projects/src/warehouse_amr_navigation/tools/test_warehouse_amr.py"

else
    echo "Beendet."
    exit 0
fi
