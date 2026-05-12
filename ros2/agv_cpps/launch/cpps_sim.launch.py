"""
cpps_sim.launch.py
──────────────────────────────────────────────────────────────────────
Master launch file for the AGV CPPS Replenishment Simulation.

Starts:
  1. Gazebo with agv_factory world
  2. order_manager  — shop inventory digital twin + order generation
  3. fleet_manager  — order-to-AGV assignment
  4. agv_controller — one per robot (agv_1, agv_2, agv_3)
  5. event_logger   — live CPPS console dashboard
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_pkg = get_package_share_directory("agv_factory_world")
    world_file = os.path.join(world_pkg, "worlds", "agv_factory.world")
    models_dir = os.path.join(world_pkg, "models")

    ign_model_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ign_model_path = models_dir + (":" + ign_model_path if ign_model_path else "")

    consume_interval = DeclareLaunchArgument(
        "consume_interval_sec", default_value="8.0",
        description="Seconds between shop stock consumption ticks"
    )

    # ── 1. Ignition Gazebo via ros_gz_sim ─────────────────────────────
    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "gz_version": "6",
        }.items(),
    )
    # Export model path so Ignition can find our custom models
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ign_model_path

    # ── 2. ROS <-> Ignition bridge ────────────────────────────────────
    # One bridge node per AGV so topic names match exactly with no remapping.
    # /agv_N/cmd_vel  ROS2→IGN  (controller → diff-drive plugin)
    # /model/agv_N/odometry IGN→ROS2  (diff-drive odom back to ROS2)
    gz_bridge = TimerAction(period=3.0, actions=[
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            arguments=[
                # cmd_vel: ROS2 → Ignition
                "/model/agv_1/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                "/model/agv_2/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                "/model/agv_3/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                # odometry: Ignition → ROS2 (kept for velocity data)
                "/model/agv_1/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/model/agv_2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/model/agv_3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                # world pose info: exact world-frame poses for every model (Ignition → ROS2)
                "/world/agv_factory/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            ],
            output="screen",
        )
    ])

    # ── 3. Order Manager ───────────────────────────────────────────────
    order_manager = TimerAction(period=3.0, actions=[
        Node(
            package="agv_cpps",
            executable="order_manager",
            name="order_manager",
            output="screen",
            parameters=[{
                "consume_interval_sec": LaunchConfiguration("consume_interval_sec"),
                "publish_interval_sec": 1.0,
            }],
        )
    ])

    # ── 3. Fleet Manager ───────────────────────────────────────────────
    fleet_manager = TimerAction(period=3.5, actions=[
        Node(
            package="agv_cpps",
            executable="fleet_manager",
            name="fleet_manager",
            output="screen",
        )
    ])

    # ── 4. AGV Controllers ─────────────────────────────────────────────
    agv_nodes = [
        TimerAction(period=4.0, actions=[
            Node(
                package="agv_cpps",
                executable="agv_controller",
                name=f"agv_controller_{i}",
                output="screen",
                parameters=[{"agv_id": f"agv_{i}"}],
            )
        ])
        for i in range(1, 4)
    ]

    # ── 5. Event Logger ────────────────────────────────────────────────
    event_logger = TimerAction(period=5.0, actions=[
        Node(
            package="agv_cpps",
            executable="event_logger",
            name="event_logger",
            output="screen",
        )
    ])

    return LaunchDescription([
        consume_interval,
        gazebo,
        gz_bridge,
        order_manager,
        fleet_manager,
        *agv_nodes,
        event_logger,
    ])
