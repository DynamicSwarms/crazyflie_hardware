import os


from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    crtp_driver_pkg_prefix = get_package_share_directory('crtp_driver')
    
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
            
    crtp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [crtp_driver_pkg_prefix, '/launch/crazyradio.launch.py']),
            launch_arguments={}.items()       
    )


    return LaunchDescription([
        crazyflie_server,
        crtp
        ])

