from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port device for the IMU sensor",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="imu_link",
        description="Frame ID for the published IMU message",
    )
    topic_name_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="/Imu_data",
        description="Topic name for the published IMU message",
    )

    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")
    topic_name = LaunchConfiguration("topic_name")

    talker_node = Node(
        package="serial_imu",
        executable="talker",
        name="serial_imu_talker",
        output="screen",
        parameters=[{
            "port": port,
            "frame_id": frame_id,
            "topic_name": topic_name,
        }],
    )

    listener_node = Node(
        package="serial_imu",
        executable="listener",
        name="serial_imu_listener",
        output="screen",
        parameters=[{
            "topic_name": topic_name,
        }],
    )

    return LaunchDescription([
        port_arg,
        frame_id_arg,
        topic_name_arg,
        talker_node,
        listener_node,
    ])


