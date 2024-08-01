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
        executable="crazyradio"
    )

    crazyradio_cpp = Node(
        package="crazyradio_cpp",
        executable="crazyradio"
    )

    radiolistener = Node(
        package="crtp_driver",
        executable="radiolistener"
    )

    id = 0
    cf = Node(
        package="crtp_driver",
        executable="crazyflie",
        parameters=[
            {"id": id},
            {"channel": 100},
            {"datarate": 2}
        ],
        name='cf' + str(id)
    )

    id = 0xE7
    cf_broad = Node(
        package="crtp_driver",
        executable="crazyflie",
        parameters=[
            {"id": id},
            {"channel": 100},
            {"datarate": 2}
        ],
        name="cf" + str(id)
    )

    return LaunchDescription([
        #crazyradio,
        crazyradio_cpp,
        radiolistener,
        cf,
        cf_broad
        ])

