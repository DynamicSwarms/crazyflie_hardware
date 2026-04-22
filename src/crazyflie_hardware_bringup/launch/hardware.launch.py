from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_radios(context):
    use_udp_radio = (
        LaunchConfiguration("sitl_udp_radio", default="false").perform(context).lower()
        == "true"
    )
    channels: str = LaunchConfiguration("radio_channels").perform(context)
    for channel in [
        int(channel_str) for channel_str in channels.strip("[]").split(",")
    ]:
        yield Node(
            package="crazyradio",
            executable="crazyradio_node",
            name=f"crazyradio{channel}",
            # prefix=["gdbserver localhost:3000"],
            parameters=[{"channel": channel, "use_udpradio": use_udp_radio}],
        )


def generate_launch_description():
    default_types_yaml = os.path.join(
        get_package_share_directory("crazyflie_hardware_bringup"),
        "config",
        "crazyflieTypes.yaml",
    )
    default_crazyflie_configuration_yaml = os.path.join(
        get_package_share_directory("crazyflie_hardware_bringup"),
        "config",
        "crazyflieConfig.yaml",
    )

    types_yaml_launch_argument = DeclareLaunchArgument(
        name="crazyflie_types_yaml",
        default_value=default_types_yaml,
        description="Path to a .yaml file which specifies different crazyflie types and their"
        + "corresponding marker and dynamics configuration index",
    )

    crazyflie_configuration_yaml_launch_argument = DeclareLaunchArgument(
        name="crazyflie_configuration_yaml",
        default_value=default_crazyflie_configuration_yaml,
        description="Path to a .yaml file which which describes crazyflie configuration"
        + "the configuration describes the default firmware parameters",
    )

    sitl_udp_radio_launch_argument = DeclareLaunchArgument(
        name="sitl_udp_radio",
        default_value="false",
        description="Whether to use the UDP radio for communication with a SITL instance. If true, the crazyradio nodes will be launched with the use_udpradio parameter set to true.",
    )

    radios_launch_argument = DeclareLaunchArgument(
        "radio_channels",
        default_value="[80]",
        description="List of crazyradios to spawn. With a different channel for each",
    )

    crazyflie_gateway = Node(
        package="crazyflie_hardware_gateway",
        executable="gateway",
        parameters=[
            LaunchConfiguration("crazyflie_types_yaml"),
            {
                "crazyflie_configuration_yaml": LaunchConfiguration(
                    "crazyflie_configuration_yaml"
                )
            },
        ],
    )

    broadcaster = Node(
        package="crazyflie_hardware",
        executable="broadcaster",
    )

    radiolistener = Node(
        package="crazyflie_hardware_examples", executable="radiolistener"
    )

    return LaunchDescription(
        [
            types_yaml_launch_argument,
            crazyflie_configuration_yaml_launch_argument,
            sitl_udp_radio_launch_argument,
            crazyflie_gateway,
            broadcaster,
            radiolistener,
            radios_launch_argument,
            OpaqueFunction(function=generate_radios),
        ]
    )
