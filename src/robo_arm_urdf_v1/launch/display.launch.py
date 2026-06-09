from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    pkg_share = FindPackageShare("robo_arm_urdf_v1")
    robot_description_topic = "/robo_arm_urdf_v1/robot_description"
    joint_states_topic = "/robo_arm_urdf_v1/joint_states"
    link_prefix = "robo_arm_urdf_v1_"

    urdf_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "robo_arm_urdf_v1.urdf.xacro"
    ])

    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        "rviz",
        "display.rviz"
    ])

    robot_description_content = Command([
        "xacro ",
        urdf_file,
        " prefix:=",
        link_prefix
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="Start joint_state_publisher_gui instead of joint_state_publisher."
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robo_arm_robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            remappings=[
                ("robot_description", robot_description_topic),
                ("joint_states", joint_states_topic)
            ]
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="robo_arm_joint_state_publisher",
            output="screen",
            parameters=[{
                "rate": 30,
                "publish_default_positions": True
            }],
            remappings=[
                ("robot_description", robot_description_topic),
                ("joint_states", joint_states_topic)
            ],
            condition=UnlessCondition(use_gui)
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="robo_arm_joint_state_publisher_gui",
            output="screen",
            parameters=[{
                "rate": 30,
                "publish_default_positions": True
            }],
            remappings=[
                ("robot_description", robot_description_topic),
                ("joint_states", joint_states_topic)
            ],
            condition=IfCondition(use_gui)
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="robo_arm_rviz2",
            output="screen",
            arguments=["-d", rviz_config_file]
        ),
    ])
