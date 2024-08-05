#!/usr/bin/env python3
import numpy as np
import math
import struct
import rclpy
from rclpy.node import Node
from broadcaster_interface.srv import PosiPoseBroadcastObject
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crtp_driver.crtplib.packers import CrtpPacker
from crtp_interface.msg import CrtpPacket
from .packer import Packer
from .crtp_link_ros import CrtpLinkRos


class Broadcaster(Node):
    def __init__(self):
        super().__init__("broadcaster")
        self.world = self.declare_parameter('world', 'world').get_parameter_value().string_value
        self.position_only = True  # self.declare_parameter('position_only', True).get_parameter_value().bool_value
        self.hz = self.declare_parameter('hz', 10).get_parameter_value().integer_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.hz, self.run)
        #self.address = (0xFF, 0xE7, 0xE7, 0xE7, 0xE7)  # TODO very hacky way to test broadcasting
        #https://github.com/USC-ACTLab/crazyswarm/blob/master/ros_ws/src/crazyswarm/src/crazyswarm_server.cpp 810 Paar infos über broadcasting Adresse
        #https://github.com/whoenig/crazyflie_cpp/blob/25bc72c120f8cea6664dd24e334eefeb7c9606ca/src/Crazyflie.cpp alter code von broadcaster

        self.crtp_link = CrtpLinkRos(self, self.channel, self.address, self.datarate)
        self.broadcaster_commander = PositionBroadcasterCommander(self, self.crtp_link)


    def run(self):
        rclpy.logging.get_logger("broadcaster").info("Running")
        self.broadcaster_commander.send_external_positions(list(self._get_external_positions()))

    def _get_external_positions(self):
        for frame in self.frames:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world,
                    frame,
                    rclpy.time.Time())
            except TransformException as ex:
                continue
            id_ = frame
            pos = [t.transform.translation.x * 1000,
                   t.transform.translation.y * 1000,
                   t.transform.translation.z * 1000]
            yield id_, pos

    def _get_id_of_frame(self, frame) -> int:
        id_ = frame.replace("cf", "")
        if id_.isdigit():
            return int(id_)
        return 0 #TODO check interactables in crazyswarm for if of non cf frames


class BroadcasterLogic:
    def __init__(self, CrtpPacker, crtp_link):
        self.link = crtp_link
        self.packer = BroadcasterPacker(CrtpPacker)
        self.frames = set()
        self.channels = set()

    def add_object(self, channel: int, frame: str) -> bool:
        if channel != 0:
            self.channels.add(channel)
        self.frames.add(frame)
        return True

    def remove_object(self, frame: str) -> bool:
        if frame in self.frames:
            self.frames.remove(frame)
            return True
        return False

    def send_external_positions(self, id_positions):
        for i in range(math.ceil(len(id_positions) / 4)):
            packets = []
            for id_, pos in id_positions[i * 4: i * 4 + 4]:
                packets.append(self.packer.create_external_position_packet(id_, *pos))
            packet = self.packer.send_external_positions(packets)
            self.link.send_packet(packet)
            #TODO send2packets in https://github.com/whoenig/crazyflie_cpp/blob/25bc72c120f8cea6664dd24e334eefeb7c9606ca/src/Crazyflie.cpp


class PositionBroadcasterCommander(BroadcasterLogic):
    def __init__(self, node, crtp_link):
        super().__init__(CrtpPacker, crtp_link)
        self.node = node
        self.add_service = node.create_service(PosiPoseBroadcastObject, "add_posi_pose_object", self._add_object)
        self.remove_service = node.create_service(PosiPoseBroadcastObject, "remove_posi_pose_object", self.remove_object)

    def _add_object(self, request, response):
        response.success = self.add_object(request, response)
        return response

    def _remove_object(self, request, response):
        response.success = self.remove_object(request.tf_frame_id)
        return response


class BroadcasterPacker(Packer):

    PORT_LOCALIZATION = 6
    POSITION_CH = 2

    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_LOCALIZATION)

    def send_external_positions(self, position_packets: list) -> CrtpPacket:
        data = b''
        for packet in position_packets[:4]:
            data += packet
        return self._prepare_packet(channel=self.POSITION_CH, data=data)

    def create_external_position_packet(self, id_, x, y, z) -> bytes:
        return struct.pack('<BHHH', id_, x, y, z)

def main(args=None):
    rclpy.init(args=args)
    node = Broadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()