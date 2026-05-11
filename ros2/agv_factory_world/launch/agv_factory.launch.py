import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("agv_factory_world")
    world_file = os.path.join(pkg, "worlds", "agv_factory.world")
    models_dir = os.path.join(pkg, "models")

    gz_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    if gz_model_path:
        gz_model_path = models_dir + ":" + gz_model_path
    else:
        gz_model_path = models_dir

    return LaunchDescription([
        DeclareLaunchArgument("gui",     default_value="true",  description="Launch Gazebo GUI"),
        DeclareLaunchArgument("verbose", default_value="false", description="Verbose Gazebo output"),

        # Start Gazebo with the factory world
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose" if LaunchConfiguration("verbose") == "true" else "",
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
                "-s", "libgazebo_ros_state.so",
                world_file,
            ],
            additional_env={
                "GAZEBO_MODEL_PATH": gz_model_path,
            },
            output="screen",
        ),

        # Robot state publishers for each AGV (remapped per namespace)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="rsp_agv1",
            namespace="agv_1",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="rsp_agv2",
            namespace="agv_2",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="rsp_agv3",
            namespace="agv_3",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
