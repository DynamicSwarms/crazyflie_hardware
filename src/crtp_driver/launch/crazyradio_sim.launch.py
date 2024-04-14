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

    cf_id = 0x10
    cf = Node(
        package="crtp_driver",
        executable="crazyflie",
        name="cf" + str(cf_id),
        parameters=[
            {"id": cf_id},
            {"channel": 101},
            {"datarate": 2}
        ]
    )

    return LaunchDescription([
        crazyradio,
        radiolistener,
        cf
        ])

