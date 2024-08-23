import os
import pathlib
import launch
import yaml
import shutil
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    crazyradio_cpp = Node(
        package="crazyradio_cpp",
        executable="crazyradio_node"
    )

    broadcaster = Node(
        package="crtp_driver",
        executable="crtp_broadcaster"
    )

    radiolistener = Node(
        package="crtp_driver",
        executable="radiolistener"
    )

#    cf_config = os.path.join(get_package_share_directory('crtp_driver'),
#                          'launch',
#                          'crazyflie_config.yaml')
#    
#    id = 0xA1
#    cf = Node(
#        package="crtp_driver",
#        executable="crazyflie",
#        parameters=[
#            {
#            "id": id,
#            "channel": 100,
#            "datarate": 2,
#            "initial_position": [0.0, 0.0, 0.0]},
#            cf_config
#        ],
#        name='cf' + str(id)
#    )
#
#    id = 0xA2
#    cf_2 = Node(
#        package="crtp_driver",
#        executable="crazyflie",
#        parameters=[
#            {
#            "id": id,
#            "channel": 100,
#            "datarate": 2,
#            "initial_position": [0.0, 1.0, 0.0]},
#            cf_config
#        ],
#        name='cf' + str(id)
#    )

    
    #id = 0xE7
    #cf_broad = Node(
    #    package="crtp_driver",
    #    executable="crazyflie",
    #    parameters=[
    #        {"id": id},
    #        {"channel": 100},
    #        {"datarate": 2}
    #    ],
    #    name="cf" + str(id)
    #)

    return LaunchDescription([
        crazyradio_cpp,
        radiolistener,
        broadcaster
    ])

