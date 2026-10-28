#!/usr/bin/env bash
# ==============================================================================
# Autonomous Warehouse AMR Navigation Stack — Master Launcher
# 2D-LiDAR SLAM, Costmaps, Nav2 & Intralogistics Mission Dispatcher
# ==============================================================================

set -e

# Allow Docker container to access host X11 display (for RViz2)
xhost +local:docker > /dev/null 2>&1 || true

# Ensure ros_container is running
if [ -z "$(docker ps -q -f name=ros_container)" ]; then
    echo "➜ Starte ros_container Docker-Container..."
    docker start ros_container > /dev/null 2>&1 || true
    sleep 1
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

# Pre-flight cleanup
echo "➜ Bereinige alte Prozesse & initialisiere FastDDS..."
docker exec ros_container bash -c "pkill -9 -f 'ros2|nav2|amcl|planner_server|controller_server|behavior_server|bt_navigator|map_server|warehouse_simulator' 2>/dev/null || true; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true"
sleep 0.5

if [ "$CHOICE" == "1" ]; then
    echo "➜ Starte autonome Warehouse-Mission mit RViz2 Visualization & Live-Fahrt..."
    docker exec -it ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      ros2 launch warehouse_amr_navigation warehouse_amr.launch.py use_rviz:=true auto_mission:=true"

elif [ "$CHOICE" == "2" ]; then
    echo "➜ Starte Nav2 Bringup mit RViz2 (Manuelle Zielvorgabe)..."
    docker exec -it ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      ros2 launch warehouse_amr_navigation warehouse_amr.launch.py use_rviz:=true auto_mission:=false"

elif [ "$CHOICE" == "3" ]; then
    echo "➜ Führe automatisierten Nav2 Regressionstest aus..."
    docker exec -it ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      python3 /root/ros-projects/src/warehouse_amr_navigation/tools/test_warehouse_amr.py"

else
    echo "Beendet."
    exit 0
fi
