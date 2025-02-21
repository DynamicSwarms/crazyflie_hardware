#!/usr/bin/env python3

import math
import struct
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from broadcaster_interfaces.srv import PosiPoseBroadcastObject
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crtp_driver.crtp_packer_ros import CrtpPackerRos
from crtp_driver.crtp_link_ros import CrtpLinkRos
from crtp_interfaces.msg import CrtpPacket

from crtp.packers.packer import Packer
from crtp.packers.crtp_packer import CrtpPacker

from crtp.logic.logic import Logic

from typing import Callable


class Broadcaster(Node):
    def __init__(self):
        super().__init__("broadcaster")
        self.world = (
            self.declare_parameter("world", "world").get_parameter_value().string_value
        )
        self.position_only = True  # self.declare_parameter('position_only', True).get_parameter_value().bool_value
        self.hz = self.declare_parameter("hz", 10).get_parameter_value().integer_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.hz, self.run)
        # https://github.com/USC-ACTLab/crazyswarm/blob/master/ros_ws/src/crazyswarm/src/crazyswarm_server.cpp 810 Paar infos über broadcasting Adresse
        # https://github.com/whoenig/crazyflie_cpp/blob/25bc72c120f8cea6664dd24e334eefeb7c9606ca/src/Crazyflie.cpp alter code von broadcaster

        self.broadcaster_commander = PositionBroadcasterCommander(self)

    def run(self):
        self.broadcaster_commander.send_external_positions(
            list(self._get_external_positions())
        )

    def _get_external_positions(self):
        for frame in self.broadcaster_commander.frame_channels.keys():
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame=self.world,
                    source_frame=frame,
                    time=rclpy.time.Time(),
                    timeout=Duration(seconds=0.01),
                )
            except TransformException as ex:
                continue
            id_ = self._get_id_of_frame(frame)
            pos = [
                t.transform.translation.x * 1000,
                t.transform.translation.y * 1000,
                t.transform.translation.z * 1000,
            ]
            yield id_, pos

    def _get_id_of_frame(self, frame) -> int:
        id_ = frame.replace("cf", "")
        if id_.isdigit():
            return int(id_)
        return 0  # TODO check interactables in crazyswarm for if of non cf frames


class BroadcasterLogic:
    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker], node: Node):
        self.node = node
        self.packer = BroadcasterPacker(crtp_packer_factory)
        self.address = (0xFF, 0xE7, 0xE7, 0xE7, 0xE7)

        self.frame_channels = {}  # to keep track of all frames and their channels
        self.crtp_links = []

    def add_object(self, channel: int, frame: str, data_rate: int = 2) -> bool:
        if channel != 0:
            if frame not in self.frame_channels.keys():
                # multiple channels can use the same id
                self.frame_channels[frame] = [channel]
            # Add a link if there is no link for this channel
            if not channel in (link.channel for link in self.crtp_links):
                self.node.get_logger().debug(f"Adding channel {channel}")
                self.crtp_links.append(
                    CrtpLinkRos(self.node, channel, self.address, data_rate, None)
                )
        return True

    def remove_object(self, channel: int, frame: str) -> bool:
        if frame in self.frame_channels.keys():
            if frame in self.frame_channels.keys():
                if len(self.frame_channels[frame]) == 1:
                    self.frame_channels.pop(frame)
                else:
                    self.frame_channels[frame].remove(channel)

                # remove link if no more frames are using it
                if not any(channel in val for val in self.frame_channels.values()):
                    self.crtp_links = [
                        link for link in self.crtp_links if link.channel != channel
                    ]
            return True
        return False

    def send_external_positions(self, id_positions):
        for i in range(math.ceil(len(id_positions) / 4)):
            packets = []
            for id_, pos in id_positions[i * 4 : i * 4 + 4]:
                # self.node.get_logger().info(f"Sending position for {id_}: {pos}")
                packets.append(
                    self.packer.create_external_position_packet(
                        id_, int(pos[0]), int(pos[1]), int(pos[2])
                    )
                )
            packet = self.packer.send_external_positions(packets)

            # send packet to all channels
            for link in self.crtp_links:
                link.send_packet_no_response(packet)
            # TODO send2packets in https://github.com/whoenig/crazyflie_cpp/blob/25bc72c120f8cea6664dd24e334eefeb7c9606ca/src/Crazyflie.cpp


class PositionBroadcasterCommander(BroadcasterLogic):
    def __init__(self, node: Node):
        super().__init__(CrtpPackerRos, node)
        self.node = node
        self.add_service = node.create_service(
            PosiPoseBroadcastObject, "add_posi_pose_object", self._add_object
        )
        self.remove_service = node.create_service(
            PosiPoseBroadcastObject, "remove_posi_pose_object", self._remove_object
        )

    def _add_object(self, request, response):
        response.success = self.add_object(
            request.channel, request.tf_frame_id, request.data_rate
        )
        return response

    def _remove_object(self, request, response):
        response.success = self.remove_object(request.channel, request.tf_frame_id)
        return response


class BroadcasterPacker(Packer):

    PORT_LOCALIZATION = 6
    POSITION_CH = 2

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_LOCALIZATION)

    def send_external_positions(self, position_packets: list) -> CrtpPacket:
        data = b""
        for packet in position_packets[:4]:
            data += packet
        return self._prepare_packet(channel=self.POSITION_CH, data=data)

    def create_external_position_packet(
        self, id_: int, x: int, y: int, z: int
    ) -> bytes:
        return struct.pack("<Bhhh", id_, x, y, z)


def main(args=None):
    rclpy.init(args=args)
    bc = Broadcaster()

    try:
        while rclpy.ok():
            rclpy.spin_once(bc, timeout_sec=1.0)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        exit()


if __name__ == "__main__":
    main()
