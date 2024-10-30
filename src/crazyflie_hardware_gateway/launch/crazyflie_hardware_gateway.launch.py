import os


from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(get_package_share_directory('crazyflie_hardware_gateway'),
                          'launch',
                          'crazyflieTypes.yaml')
    
    crazyflie_gateway = Node(
        package="crazyflie_hardware_gateway",
        executable="gateway", 
        parameters = [
            config
        ]
    )

    crazyradio = Node(
        package="crazyradio",
        executable="crazyradio_node"
    )

    broadcaster = Node(
        package="crazyflie_hardware",
        executable="broadcaster"
    )

    radiolistener = Node(
        package="crazyflie_hardware_examples",
        executable="radiolistener"
    )


    return LaunchDescription([
        crazyflie_gateway, 
        crazyradio, 
        broadcaster,
        radiolistener
        ])

