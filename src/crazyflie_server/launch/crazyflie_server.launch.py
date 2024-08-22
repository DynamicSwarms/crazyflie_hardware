import os


from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_server')
    
    config = os.path.join(get_package_share_directory('crazyflie_server'),
                          'launch',
                          'crazyflieTypes.yaml')
    
    crazyflie_server = Node(
        package="crazyflie_server",
        executable="server", 
        parameters = [
            config
        ]
    )

    return LaunchDescription([
        crazyflie_server
        ])

