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
            srv_type=Crazyflie,
            srv_name="~/add_crazyflie",
            callback=self._add_crazyflie_callback,
        )
        self.remove_service = self.create_service(
            srv_type=Crazyflie,
            srv_name="~/remove_crazyflie",
            callback=self._remove_crazyflie_callback,
        )

    def remove_crazyflie(self, channel: int, id: int) -> bool:
        """Remove a crazyflie.

        Removes the crazyflie. This Stops the Crazyflies process with a SIGINT.
        The crazyflie then automatically closes its connection to the radio.

        Args:
            channel (int): The channel of the crazyflie
            id (int): The id of the crazyflie

        Returns:
            bool: Returns False if crazyflie was not in the list of started crazyflies.
        """
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
        """Removes all crazyflies which were started with the gateway."""
        for cf_key in list(self.crazyflies.keys()):
            _ = self.remove_crazyflie(*cf_key)
            del self.crazyflies[cf_key]

    def add_crazyflie(
        self,
        channel: int,
        id: int,
        initial_position: Point,
        type: str,
    ) -> bool:
        """Adds a crazyflie with given configuration.

        Creates and holds a process to a crazyflie.

        Args:
            channel (int): The channel the crayflie is on
            id (int): The id of the crazyflie
            initial_position (Point): The expected initial position
            type (str): The type as referencened in a .yaml (stores marker and dynamics Conf idx...)
        Returns:
            bool: True if the crazyflie could be created. False if we already have a crazyflie with same id/channel
        """
        self._logger.info("Got called to add Crazyflie with ID: {}".format(id))
        if (channel, id) in self.crazyflies.keys():
            self._logger.info("Cannot add Crazyflie, is already in Gateway")
            return False
        wait_future = asyncio.ensure_future(
            self._create_cf(
                channel,
                id,
                initial_position,
                type,
            )
        )
        wait_future.add_done_callback(
            lambda fut: self._on_crazyflie_exit(fut, channel, id)
        )
        return True

    def _on_crazyflie_exit(self, fut: asyncio.Future, channel: int, id: int):
        """Callback invoked when a crazyflie subprocess exits."""
        self._logger.info(f"Crazyflie (channel={channel}, id={id}) exited.")

        # Clean up the crazyflie entry from the dictionary
        if (channel, id) in self.crazyflies:
            del self.crazyflies[(channel, id)]

    async def _create_cf(
        self,
        channel: int,
        id: int,
        initial_position: List[float],
        type: str,
    ):
        cmd = self._create_start_command(
            channel,
            id,
            initial_position,
            type,
        )
        self.crazyflies[(channel, id)] = await asyncio.create_subprocess_exec(
            *cmd, preexec_fn=os.setsid
        )
        await self.crazyflies[(channel, id)].wait()

    def _create_start_command(
        self,
        channel: int,
        id: int,
        initial_position: List[float],
        type: str,
    ) -> List[str]:
        crazyflie_path = get_executable_path(
            package_name="crazyflie_hardware", executable_name="crazyflie"
        )

        send_external_position = self.__get_send_external_position(type)
        send_external_pose = self.__get_send_external_pose(type)
        max_initial_deviation = self.__get_max_initial_deviation(type)
        marker_configuration_index = self.__get_marker_configuration_index(type)
        dynamics_configuration_index = self.__get_dynamics_configuration_index(type)

        cmd = [crazyflie_path, "--ros-args"]

        def add_parameter(name: str, value: str):
            cmd.append("-p")
            cmd.append(f"{name}:={value}")

        add_parameter("id", str(id))
        add_parameter("channel", str(channel))
        add_parameter("datarate", str(2))
        add_parameter("initial_position", "[{},{},{}]".format(*initial_position))
        add_parameter("send_external_position", str(send_external_position))
        add_parameter("send_external_pose", str(send_external_pose))
        add_parameter("max_initial_deviation", str(max_initial_deviation))
        add_parameter("marker_configuration_index", str(marker_configuration_index))
        add_parameter("dynamics_configuration_index", str(dynamics_configuration_index))

        cmd += [
            "--params-file",
            self.crazyflie_configuration_yaml,
            "-r",
            "__node:=cf{}".format(id),
        ]

        return cmd

    def _add_crazyflie_callback(
        self, req: Crazyflie.Request, resp: Crazyflie.Response
    ) -> Crazyflie.Response:
        resp.success = self.add_crazyflie(
            req.channel,
            req.id,
            [req.initial_position.x, req.initial_position.y, req.initial_position.z],
            req.type,
        )

        return resp

    def _remove_crazyflie_callback(
        self, req: Crazyflie.Request, resp: Crazyflie.Response
    ) -> Crazyflie.Response:
        resp.success = self.remove_crazyflie(req.channel, req.id)
        return resp

    def __get_send_external_position(self, type: str) -> bool:
        name = "sendExternalPosition"
        return self.__get_typed_parameter_value(type, name).bool_value

    def __get_send_external_pose(self, type: str) -> bool:
        name = "sendExternalPose"
        return self.__get_typed_parameter_value(type, name).bool_value

    def __get_max_initial_deviation(self, type: str) -> float:
        name = "maxInitialDeviation"
        return self.__get_typed_parameter_value(type, name).double_value

    def __get_marker_configuration_index(self, type: str) -> int:
        name = "markerConfigurationIndex"
        return self.__get_typed_parameter_value(type, name).integer_value

    def __get_dynamics_configuration_index(self, type: str) -> int:
        name = "dynamicsConfigurationIndex"
        return self.__get_typed_parameter_value(type, name).integer_value

    def __get_typed_parameter_value(self, type: str, name: str):
        return self.get_parameter(f"crazyflieTypes.{type}.{name}").get_parameter_value()

    @property
    def crazyflie_configuration_yaml(self) -> str:
        return (
            self.get_parameter("crazyflie_configuration_yaml")
            .get_parameter_value()
            .string_value
        )


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
