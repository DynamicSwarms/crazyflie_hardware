from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_radios(context):
    channels: str = LaunchConfiguration("radio_channels").perform(context)
    for channel in [int(channel_str) for channel_str in channels.strip("[]").split(",")]:
        yield Node(
            package="crazyradio",
            executable="crazyradio_node",
            name=f"crazyradio{channel}",
            parameters=[{"channel": channel}])


def generate_launch_description():
    default_types_yaml = os.path.join(
        get_package_share_directory("crazyflie_hardware_gateway"),
        "launch",
        "crazyflieTypes.yaml",
    )
    default_crazyflie_configuration_yaml = os.path.join(
        get_package_share_directory("crazyflie_hardware"),
        "launch",
        "crazyflie_config.yaml",
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

    types_yaml = LaunchConfiguration("crazyflie_types_yaml")
    configuration_yaml = LaunchConfiguration("crazyflie_configuration_yaml")

    crazyflie_gateway = Node(
        package="crazyflie_hardware_gateway",
        executable="gateway",
        parameters=[types_yaml, {"crazyflie_configuration_yaml": configuration_yaml}],
    )

    radios_arg = DeclareLaunchArgument('radio_channels', default_value='[80]', description="List of crazyradios to spawn. With a different channel for each")
    
    broadcaster = Node(package="crazyflie_hardware_cpp", executable="broadcaster")

    radiolistener = Node(
        package="crazyflie_hardware_examples", executable="radiolistener"
    )

    return LaunchDescription(
        [
            types_yaml_launch_argument,
            crazyflie_configuration_yaml_launch_argument,
            crazyflie_gateway,
            broadcaster,
            radiolistener,
            radios_arg,
            OpaqueFunction(function=generate_radios)
        ]
    )
