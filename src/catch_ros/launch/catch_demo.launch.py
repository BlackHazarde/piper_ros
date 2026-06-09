from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value="/home/artorias/work/vision_model/best.pt"),
            DeclareLaunchArgument("target_class", default_value="red"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("enable_arm", default_value="false"),
            DeclareLaunchArgument("plane_fit", default_value="true"),
            DeclareLaunchArgument("show_window", default_value="true"),
            DeclareLaunchArgument("top_normal_threshold", default_value="0.65"),
            DeclareLaunchArgument("side_normal_threshold", default_value="0.45"),
            DeclareLaunchArgument("publish_moveit_target", default_value="true"),
            DeclareLaunchArgument("moveit_target_pose_topic", default_value="/catch/top_pregrasp_pose"),
            Node(
                package="catch_ros",
                executable="catch_node",
                name="catch_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "target_class": LaunchConfiguration("target_class"),
                        "execute": LaunchConfiguration("execute"),
                        "enable_arm": LaunchConfiguration("enable_arm"),
                        "plane_fit": LaunchConfiguration("plane_fit"),
                        "show_window": LaunchConfiguration("show_window"),
                        "top_normal_threshold": LaunchConfiguration("top_normal_threshold"),
                        "side_normal_threshold": LaunchConfiguration("side_normal_threshold"),
                        "publish_moveit_target": LaunchConfiguration("publish_moveit_target"),
                        "moveit_target_pose_topic": LaunchConfiguration("moveit_target_pose_topic"),
                    }
                ],
            ),
        ]
    )
