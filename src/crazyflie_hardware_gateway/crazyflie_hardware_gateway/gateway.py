#!/usr/bin/env python3
import os
import asyncio
from asyncio.subprocess import Process
from signal import SIGINT

from ament_index_python.packages import get_package_share_directory
from ros2run.api import get_executable_path

import rclpy
from rclpy.node import Node

from crazyflie_hardware_gateway_interfaces.srv import Crazyflie
from geometry_msgs.msg import Point

from typing import Dict, Tuple, List


class Gateway(Node):

    def __init__(self):
        super().__init__(
            "crazyflie_hardware_gateway",
            automatically_declare_parameters_from_overrides=True,
        )
        self._logger = self.get_logger()
        self._logger.info("Started Hardware-Gateway")

        self.crazyflies: Dict[Tuple[int, int], Process] = {}

        self.add_service = self.create_service(
            Crazyflie, "~/add_crazyflie", self._add_crazyflie_callback
        )
        self.remove_service = self.create_service(
            Crazyflie, "~/remove_crazyflie", self._remove_crazyflie_callback
        )

    def remove_crazyflie(self, id: int, channel: int) -> bool:
        self.get_logger().info("Removing Crazyflie with ID: {}".format(id))
        if (channel, id) in self.crazyflies.keys():
            process = self.crazyflies[(channel, id)]
            os.killpg(os.getpgid(process.pid), SIGINT)
            return True
        self._logger.info(
            "Couldn't remove crazyflie with Channel: {}, ID: {}; was not active".format(
                channel, id
            )
        )
        return False

    def remove_all_crazyflies(self):
        """
        TODO: We should delete the entries in the dict. But this change self reference stuff
        """
        for cf_key in self.crazyflies.keys():
            _ = self.remove_crazyflie(*cf_key)

    def add_crazyflie(
        self, channel: int, id: int, initial_position: Point, type: str
    ) -> bool:
        self._logger.info("Got called to add Crazyflie with ID: {}".format(id))
        if (channel, id) in self.crazyflies.keys():
            self._logger.info("Cannot add Crazyflie, is already in Gateway")
        asyncio.ensure_future(self._create_cf(channel, id, initial_position, type))
        return True

    async def _create_cf(
        self, channel: int, id: int, initial_position: List[float], type: str
    ):
        cmd = self._create_start_command(channel, id, initial_position)
        self.crazyflies[(channel, id)] = await asyncio.create_subprocess_exec(
            *cmd, preexec_fn=os.setsid
        )
        await self.crazyflies[(channel, id)].wait()

    def _create_start_command(
        self, channel: int, id: int, initial_position: List[float]
    ) -> List[str]:
        crazyflie_path = get_executable_path(
            package_name="crazyflie_hardware", executable_name="crazyflie"
        )

        markerConfigurationIdx = (
            self.get_parameter("crazyflieTypes." + type + ".markerConfiguration")
            .get_parameter_value()
            .integer_value
        )
        dynamicsConfigurationIdx = (
            self.get_parameter("crazyflieTypes." + type + ".dynamicsConfiguration")
            .get_parameter_value()
            .integer_value
        )

        initial_position: List[float] = [
            initial_position.x,
            initial_position.y,
            initial_position.z,
        ]

        cf_config = os.path.join(
            get_package_share_directory("crazyflie_hardware"),
            "launch",
            "crazyflie_config.yaml",
        )

        return [
            crazyflie_path,
            "--ros-args",
            "-p",
            "id:={}".format(id),
            "-p",
            "channel:={}".format(channel),
            "-p",
            "datarate:={}".format(2),
            "-p",
            "initial_position:=[{},{},{}]".format(
                initial_position[0], initial_position[1], initial_position[2]
            ),
            "--params-file",
            cf_config,
            "-r",
            "__node:=cf{}".format(id),
        ]

    def _add_crazyflie_callback(
        self, req: Crazyflie.Request, resp: Crazyflie.Response
    ) -> Crazyflie.Response:
        resp.success = self.add_crazyflie(
            req.channel, req.id, req.initial_position, req.type.data
        )
        return resp

    def _remove_crazyflie_callback(
        self, req: Crazyflie.Request, resp: Crazyflie.Response
    ) -> Crazyflie.Response:
        resp.success = self.remove_crazyflie(req.channel, req.id)
        return resp


async def run_node():
    rclpy.init()
    gateway = Gateway()
    try:
        while rclpy.ok():
            await asyncio.sleep(0.01)
            rclpy.spin_once(gateway, timeout_sec=0)
        rclpy.shutdown()
    except asyncio.CancelledError:
        gateway.remove_all_crazyflies()


def main():
    event_loop = asyncio.get_event_loop()
    node = asyncio.ensure_future(run_node())
    try:
        event_loop.run_forever()
    except KeyboardInterrupt:
        node.cancel()
        event_loop.run_until_complete(node)
    finally:
        event_loop.close()


if __name__ == "__main__":
    main()
