from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robo_arm_urdf_v1", package_name="robo_arm_urdf_v1_moveit_config")
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    generate_state_publishers(ld, moveit_config)
    generate_ros2_control_launch(ld, moveit_config)
    generate_move_group_launch(ld, moveit_config)
    generate_moveit_rviz_launch(ld, moveit_config)
    return ld


def generate_state_publishers(ld, moveit_config):
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robo_arm_robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
    )

def generate_ros2_control_launch(ld, moveit_config):
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            output="screen",
        )
    )

    for controller in ["arm_controller", "joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )


def generate_move_group_launch(ld, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": ":0"},
    )


def generate_moveit_rviz_launch(ld, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.to_dict(),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    ))
