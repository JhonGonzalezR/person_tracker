from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    detected_person_topic = LaunchConfiguration("detected_person_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    inference_rate_hz = LaunchConfiguration("inference_rate_hz")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_motion = LaunchConfiguration("enable_motion")
    target_timeout_secs = LaunchConfiguration("target_timeout_secs")
    robot_name = LaunchConfiguration("robot_name")
    start_spot = LaunchConfiguration("start_spot")

    tracker = Node(
        package="person_tracker",
        executable="detect_person",
        parameters=[{
            "image_topic": image_topic,
            "detected_person_topic": detected_person_topic,
            "debug_image_topic": debug_image_topic,
            "inference_rate_hz": inference_rate_hz,
        }],
    )

    follow = Node(
        package="person_tracker",
        executable="follow_person",
        parameters=[{
            "detected_person_topic": detected_person_topic,
            "cmd_vel_topic": cmd_vel_topic,
            "enable_motion": enable_motion,
            "target_timeout_secs": target_timeout_secs,
        }],
    )

    spot_manager = Node(
        package="person_tracker",
        executable="spot_robot_manager",
        condition=IfCondition(start_spot),
        parameters=[{
            "robot_name": robot_name,
            "auto_claim": True,
            "auto_power_on": True,
            "auto_stand": True,
            "stop_on_shutdown": True,
            "sit_on_shutdown": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/person_tracker/input_image"),
        DeclareLaunchArgument("detected_person_topic", default_value="/person_tracker/detected_person"),
        DeclareLaunchArgument("debug_image_topic", default_value="/person_tracker/debug_image"),
        DeclareLaunchArgument("inference_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/person_tracker/cmd_vel"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("target_timeout_secs", default_value="2.0"),
        DeclareLaunchArgument("robot_name", default_value=""),
        DeclareLaunchArgument("start_spot", default_value="false"),
        spot_manager,
        tracker,
        follow,
    ])
