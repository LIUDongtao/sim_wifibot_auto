#!/usr/bin/env python3

import subprocess
import time
import os

HOME = os.path.expanduser("~")
WS = f"{HOME}/wifibot_ws"


def open_terminal(title, command):
    full_command = f"""
echo '========================================'
echo ' {title}'
echo '========================================'
{command}
echo
echo '[Process exited. Press Enter to close.]'
read
"""
    subprocess.Popen([
        "gnome-terminal",
        "--title", title,
        "--",
        "bash", "-c", full_command
    ])


# ============================================================
# 0. Generate URDF
# ============================================================

print("Generating /tmp/wifibot.urdf ...")

subprocess.run(
    f"""
    source /opt/ros/humble/setup.bash
    source {WS}/install/setup.bash
    xacro {WS}/src/wifibot_description/urdf/wifibot.urdf.xacro \
        > /tmp/wifibot.urdf
    """,
    shell=True,
    executable="/bin/bash",
    check=True
)

print("URDF generated.")


# ============================================================
# Terminal 1 - Ignition Gazebo
# ============================================================

open_terminal(
    "1 - Ignition Gazebo",
    f"""
cd {WS}
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

ign gazebo src/wifibot_gazebo/worlds/indoor.world
"""
)

print("Waiting for Gazebo to start...")
time.sleep(6)


# ============================================================
# Terminal 2 - Spawn Wifibot
# ============================================================

open_terminal(
    "2 - Spawn Wifibot",
    """
source /opt/ros/humble/setup.bash

ros2 run ros_gz_sim create \
  -world wifibot_indoor_world \
  -name wifibot \
  -file /tmp/wifibot.urdf \
  -x 0 \
  -y 0 \
  -z 0.2
"""
)

time.sleep(2)


# ============================================================
# Terminal 3 - RGB + Depth + CameraInfo + Odom + Clock Bridge
# ============================================================

open_terminal(
    "3 - Sensor Clock Bridge",
    """
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
'/world/wifibot_indoor_world/model/wifibot/link/base_footprint/sensor/zed2i_rgb_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
'/world/wifibot_indoor_world/model/wifibot/link/base_footprint/sensor/zed2i_rgb_sensor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo' \
'/world/wifibot_indoor_world/model/wifibot/link/base_footprint/sensor/zed2i_rgbd_sensor/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image' \
'/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
--ros-args \
-r /world/wifibot_indoor_world/model/wifibot/link/base_footprint/sensor/zed2i_rgb_sensor/image:=/camera/zed2i/image_raw \
-r /world/wifibot_indoor_world/model/wifibot/link/base_footprint/sensor/zed2i_rgb_sensor/camera_info:=/camera/zed2i/camera_info \
-r /world/wifibot_indoor_world/model/wifibot/link/base_footprint/sensor/zed2i_rgbd_sensor/depth_image:=/camera/zed2i/depth/image_raw
"""
)


# ============================================================
# Terminal 4 - Robot State Publisher
# ============================================================

open_terminal(
    "4 - Robot State Publisher",
    """
source /opt/ros/humble/setup.bash

ros2 run robot_state_publisher robot_state_publisher \
  /tmp/wifibot.urdf \
  --ros-args \
  -p use_sim_time:=true
"""
)


# ============================================================
# Terminal 5 - Gazebo Dynamic TF Bridge
# ============================================================


open_terminal(
    "5 - Ground Truth Pose Bridge",
    """
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
'/world/wifibot_indoor_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
"""
)

time.sleep(2)

# ============================================================
# Terminal 6 - Ground Truth Odometry
# ============================================================

open_terminal(
    "6 - Ground Truth Odometry",
    f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

python3 {WS}/ground_truth_odom.py
"""
)


# ============================================================
# Terminal 6 - RGB Camera Static TF
# ============================================================

open_terminal(
    "6 - RGB Camera TF",
    """
source /opt/ros/humble/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 \
  --y 0 \
  --z 0 \
  --roll 0 \
  --pitch 0 \
  --yaw 0 \
  --frame-id camera_link \
  --child-frame-id wifibot/base_footprint/zed2i_rgb_sensor
"""
)


# ============================================================
# Terminal 7 - Depth Camera Static TF
# ============================================================

open_terminal(
    "7 - Depth Camera TF",
    """
source /opt/ros/humble/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 \
  --y 0 \
  --z 0 \
  --roll 0 \
  --pitch 0 \
  --yaw 0 \
  --frame-id camera_link \
  --child-frame-id wifibot/base_footprint/zed2i_rgbd_sensor
"""
)


# ============================================================
# Terminal 8 - CMD_VEL Bridge
# ============================================================

open_terminal(
    "8 - CMD VEL Bridge",
    f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
"""
)


# Give bridges / TF some time to initialize
time.sleep(3)


# ============================================================
# Terminal 9 - RTAB-Map LOCALIZATION
# Load existing rtabmap.db
# ============================================================

open_terminal(
    "9 - RTAB Map Localization",
    f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  database_path:={WS}/src/wifibot_navigation/maps/indoor/rtabmap.db \
  rgb_topic:=/camera/zed2i/image_raw \
  depth_topic:=/camera/zed2i/depth/image_raw \
  camera_info_topic:=/camera/zed2i/camera_info \
  frame_id:=base_link \
  odom_topic:=/odom \
  subscribe_odom:=true \
  visual_odometry:=false \
  approx_sync:=true \
  use_sim_time:=true \
  rtabmap_args:="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true --RGBD/StartAtOrigin true --Reg/Force3DoF true --Optimizer/GravitySigma 0"
"""
)

print()
print("==========================================")
print(" Wifibot startup commands launched.")
print("==========================================")
print()
print("Terminal 1 : Ignition Gazebo")
print("Terminal 2 : Spawn Wifibot")
print("Terminal 3 : Sensor/Odom/Clock Bridge")
print("Terminal 4 : Robot State Publisher")
print("Terminal 5 : Dynamic TF Bridge")
print("Terminal 6 : RGB Static TF")
print("Terminal 7 : Depth Static TF")
print("Terminal 8 : CMD_VEL Bridge")
print("Terminal 9 : RTAB-Map")
