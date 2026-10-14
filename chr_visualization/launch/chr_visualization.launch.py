from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = Path(get_package_share_directory("chr_description"))
    visualization_share = Path(get_package_share_directory("chr_visualization"))
    robot_description = (description_share / "urdf" / "chr.urdf").read_text(
        encoding="utf-8"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    preview_without_data = LaunchConfiguration("preview_without_data")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the rosbag /clock topic",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Open RViz with the CHR layout",
            ),
            DeclareLaunchArgument(
                "preview_without_data",
                default_value="true",
                description="Show a valid zero-joint model before hardware data arrives",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="chr_robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[("/joint_states", "/chr/visualization/joint_states")],
            ),
            Node(
                package="chr_visualization",
                executable="chr_rviz",
                name="chr_rviz",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {
                        "preview_without_data": ParameterValue(
                            preview_without_data, value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="chr_rviz2",
                output="screen",
                arguments=[
                    "-d",
                    str(visualization_share / "config" / "chr_visualization.rviz"),
                ],
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
