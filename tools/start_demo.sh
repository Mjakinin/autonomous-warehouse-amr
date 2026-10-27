#!/usr/bin/env bash
# ==============================================================================
# Autonomous Warehouse AMR Navigation Stack — Master Launcher
# 2D-LiDAR SLAM, Costmaps, Nav2 & Intralogistics Mission Dispatcher
# ==============================================================================

set -e

# Allow Docker container to access host X11 display (for RViz2)
xhost +local:docker > /dev/null 2>&1 || true

echo "=============================================================================="
echo "    AUTONOMOUS WAREHOUSE AMR (AGV) — INTRA-LOGISTICS NAVIGATION STACK"
echo "=============================================================================="
echo " Bitte wähle eine Option:"
echo " [1] Interaktive Live-Demo (Nav2 Stack + RViz2 GUI + Manuelle Zielvorgabe)"
echo " [2] Vollautomatischer Flotten-Auftrag (Pick -> Highway -> Shipping -> Charger)"
echo " [3] Automatisierter Regressionstest & Benchmark (Headless)"
echo " [4] Beenden"
echo "=============================================================================="
read -p "Auswahl [1-4] (Standard: 2): " CHOICE
CHOICE="${CHOICE:-2}"

if [ "$CHOICE" == "1" ]; then
    echo "➜ Starte Nav2 Bringup mit RViz2 Visualization..."
    docker exec -it ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      ros2 launch warehouse_amr_navigation warehouse_amr.launch.py use_rviz:=true auto_mission:=false"

elif [ "$CHOICE" == "2" ]; then
    echo "➜ Starte autonome Warehouse-Mission mit Telemetrie-Dashboard..."
    docker exec -it ros_container bash -c "\
      source /opt/ros/humble/setup.bash && \
      source /root/ros-projects/install/setup.bash && \
      ros2 launch warehouse_amr_navigation warehouse_amr.launch.py use_rviz:=true auto_mission:=true"

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
