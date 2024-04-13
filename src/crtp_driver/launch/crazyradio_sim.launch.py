import os
import pathlib
import launch
import yaml
import shutil
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('crtp_driver')
    
    
    crazyradio = Node(
        package="crtp_driver",
        executable="crazyradio_sim"
    )
    radiolistener = Node(
        package="crtp_driver",
        executable="radiolistener"
    )

    cf = Node(
        package="crtp_driver",
        executable="crazyflie"
    )

    return LaunchDescription([
        crazyradio,
        radiolistener,
        cf
        ])

