#!/usr/bin/env python3
import os
import asyncio

import rclpy
from rclpy.node import Node

from crazyflie_server_interfaces.srv import Crazyflie

from ament_index_python.packages import get_package_share_directory
from ros2run.api import get_executable_path


class Server(Node):

    def __init__(self):
        super().__init__("crazyflie_server", automatically_declare_parameters_from_overrides=True)

        self.add_service = self.create_service(Crazyflie, "add_crazyflie", self.add_crazyflie)
        self.remove_service = self.create_service(Crazyflie, "remove_crazyflie", self.remove_crazyflie)

        self.cfs = {}

    def remove_crazyflie(self, req, resp):
        resp.success = True
        return resp

    def add_crazyflie(self, req, resp):
        self.get_logger().info("Got called to add Crazyflie with ID: {}".format(req.id))

        type = req.type.data
        markerConfigurationIdx = self.get_parameter("crazyflieTypes." + type + ".markerConfiguration").get_parameter_value().integer_value
        dynamicsConfigurationIdx = self.get_parameter("crazyflieTypes." + type + ".dynamicsConfiguration").get_parameter_value().integer_value

        initial_position = [req.initial_position.x,
                            req.initial_position.y,
                            req.initial_position.z
                            ]

        cf_config = os.path.join(get_package_share_directory('crtp_driver'),
                          'launch',
                          'crazyflie_config.yaml')
        args = [
            '--ros-args',
            '-p', 'id:={}'.format(req.id),
            '-p', 'channel:={}'.format(req.channel),
            '-p', 'datarate:={}'.format(2),
            '-p', 'initial_position:=[{},{},{}]'.format(initial_position[0], initial_position[1], initial_position[2]),
            '--params-file', cf_config,
            '-r', '__node:=cf{}'.format(req.id)
        ]

        self.cfs[(req.channel, req.id)] = asyncio.ensure_future(self.start_cf(args))

        resp.success = True 
        return resp
    
    async def start_cf(self, args):
        path = get_executable_path(package_name="crtp_driver", executable_name="crazyflie")
        cmd = ' '.join([path] + args)
        p =  await asyncio.create_subprocess_shell(cmd)
        await p.wait()    

async def run_node():
    server = Server()
    while rclpy.ok():
        await asyncio.sleep(0.01)
        rclpy.spin_once(server, timeout_sec=0)
    rclpy.shutdown()

def main():
    rclpy.init()
    event_loop = asyncio.get_event_loop()
    asyncio.ensure_future(run_node())
    event_loop.run_forever()

if __name__ == '__main__':
    main()