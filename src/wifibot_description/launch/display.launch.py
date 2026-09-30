import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------
    description_share = get_package_share_directory(
        'wifibot_description'
    )

    gazebo_share = get_package_share_directory(
        'wifibot_gazebo'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'wifibot.urdf.xacro'
    )

    world_file = os.path.join(
        gazebo_share,
        'worlds',
        'indoor.world'
    )

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    # ---------------------------------------------------------
    # Gazebo
    # -r = start running automatically
    # ---------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            world_file
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Robot state publisher
    # ---------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    # ---------------------------------------------------------
    # Spawn Wifibot into Gazebo
    # Wait a little so Gazebo can initialize first
    # ---------------------------------------------------------
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-world', 'wifibot_indoor_world',
                    '-name', 'wifibot',
                    '-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.2'
                ]
            )
        ]
    )

    # ---------------------------------------------------------
    # Gazebo -> ROS bridges
    # ---------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wifibot_bridge',
        output='screen',
        arguments=[
            '/world/wifibot_indoor_world/model/wifibot/link/'
            'base_footprint/sensor/zed2i_rgb_sensor/image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',

            '/world/wifibot_indoor_world/model/wifibot/link/'
            'base_footprint/sensor/zed2i_rgb_sensor/camera_info'
            '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            '/world/wifibot_indoor_world/model/wifibot/link/'
            'base_footprint/sensor/zed2i_rgbd_sensor/depth_image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',

            '/odom'
            '@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

            '/clock'
            '@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            (
                '/world/wifibot_indoor_world/model/wifibot/link/'
                'base_footprint/sensor/zed2i_rgb_sensor/image',
                '/camera/zed2i/image_raw'
            ),
            (
                '/world/wifibot_indoor_world/model/wifibot/link/'
                'base_footprint/sensor/zed2i_rgb_sensor/camera_info',
                '/camera/zed2i/camera_info'
            ),
            (
                '/world/wifibot_indoor_world/model/wifibot/link/'
                'base_footprint/sensor/zed2i_rgbd_sensor/depth_image',
                '/camera/zed2i/depth/image_raw'
            ),
        ]
    )

    # ---------------------------------------------------------
    # Dynamic TF from Gazebo:
    # odom -> base_footprint
    # ---------------------------------------------------------
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        output='screen',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ]
    )

    # ---------------------------------------------------------
    # Gazebo RGB sensor frame -> ROS camera frame
    # ---------------------------------------------------------
    rgb_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgb_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id',
            'wifibot/base_footprint/zed2i_rgb_sensor'
        ]
    )

    # ---------------------------------------------------------
    # Gazebo depth sensor frame -> ROS camera frame
    # ---------------------------------------------------------
    depth_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id',
            'wifibot/base_footprint/zed2i_rgbd_sensor'
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        tf_bridge,
        rgb_sensor_tf,
        depth_sensor_tf,
    ])
