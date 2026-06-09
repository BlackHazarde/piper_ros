from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_with_gripper_moveit"
    ).to_moveit_configs()

    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("plan_on_start", default_value="false"),
            DeclareLaunchArgument("target_pose_topic", default_value="/catch/top_pregrasp_pose"),
            DeclareLaunchArgument("planning_group", default_value="arm"),
            DeclareLaunchArgument("end_effector_link", default_value="link6"),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            DeclareLaunchArgument("planning_attempts", default_value="5"),
            DeclareLaunchArgument("max_velocity_scaling", default_value="0.20"),
            DeclareLaunchArgument("max_acceleration_scaling", default_value="0.20"),
            DeclareLaunchArgument("position_tolerance", default_value="0.01"),
            DeclareLaunchArgument("orientation_tolerance", default_value="0.10"),
            DeclareLaunchArgument("min_plan_interval_sec", default_value="1.0"),
            DeclareLaunchArgument("target_position_delta", default_value="0.02"),
            DeclareLaunchArgument("target_orientation_delta", default_value="0.15"),
            DeclareLaunchArgument("target_x", default_value="0.20"),
            DeclareLaunchArgument("target_y", default_value="0.00"),
            DeclareLaunchArgument("target_z", default_value="0.32"),
            DeclareLaunchArgument("target_roll", default_value="0.0"),
            DeclareLaunchArgument("target_pitch", default_value="1.5708"),
            DeclareLaunchArgument("target_yaw", default_value="0.0"),
            Node(
                package="catch_moveit_demo",
                executable="catch_moveit_planner",
                name="catch_moveit_planner",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "execute": LaunchConfiguration("execute"),
                        "plan_on_start": LaunchConfiguration("plan_on_start"),
                        "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                        "planning_group": LaunchConfiguration("planning_group"),
                        "end_effector_link": LaunchConfiguration("end_effector_link"),
                        "planning_time": LaunchConfiguration("planning_time"),
                        "planning_attempts": LaunchConfiguration("planning_attempts"),
                        "max_velocity_scaling": LaunchConfiguration("max_velocity_scaling"),
                        "max_acceleration_scaling": LaunchConfiguration("max_acceleration_scaling"),
                        "position_tolerance": LaunchConfiguration("position_tolerance"),
                        "orientation_tolerance": LaunchConfiguration("orientation_tolerance"),
                        "min_plan_interval_sec": LaunchConfiguration("min_plan_interval_sec"),
                        "target_position_delta": LaunchConfiguration("target_position_delta"),
                        "target_orientation_delta": LaunchConfiguration("target_orientation_delta"),
                        "target_x": LaunchConfiguration("target_x"),
                        "target_y": LaunchConfiguration("target_y"),
                        "target_z": LaunchConfiguration("target_z"),
                        "target_roll": LaunchConfiguration("target_roll"),
                        "target_pitch": LaunchConfiguration("target_pitch"),
                        "target_yaw": LaunchConfiguration("target_yaw"),
                    }
                ],
            ),
        ]
    )
