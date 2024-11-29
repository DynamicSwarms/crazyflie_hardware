import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from crtp_interfaces.srv import CrtpPacketSend
from crtp_interfaces.msg import CrtpLinkEnd
from crtp_interfaces.msg import CrtpPacket
from crtp_interfaces.msg import CrtpResponse


from crtp.crtp_link import CrtpLink
from typing import Callable, List, Dict


class CrtpLinkRos(CrtpLink):
    def __init__(
        self,
        node: Node,
        channel: int,
        id: int,
        datarate: int,
        link_end_callback: Callable[[], None],
    ):
        super().__init__(channel, id, datarate)
        self.node: Node = node
        callback_group = MutuallyExclusiveCallbackGroup()
        self.link_end_callback: Callable[[],] = link_end_callback

        qos_profile: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.send_packet_service = node.create_client(
            CrtpPacketSend,
            "/crazyradio/send_crtp_packet",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        while not self.send_packet_service.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(
                "Send CRTP Packet Service not available, waiting again..."
            )

        node.create_subscription(
            CrtpResponse,
            "crazyradio/crtp_response",
            self._handle_crtp_response,
            10,
            callback_group=callback_group,
        )

        # The Link end has to be in seperate callback group
        node.create_subscription(
            msg_type=CrtpLinkEnd,
            topic="/crazyradio/crtp_link_end",
            callback=self._crtp_link_end_callback,
            qos_profile=10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.callbacks: Dict[int, List[Callable[[CrtpPacket], None]]] = {}

    def add_callback(self, port: int, callback: Callable[[CrtpPacket], None]):
        if port not in self.callbacks.keys():
            self.callbacks[port] = []
        self.callbacks[port].append(callback)

    def _handle_crtp_response(self, msg: CrtpResponse):
        if msg.channel == self.channel and (msg.address == self.address).all():
            if msg.packet.port in self.callbacks.keys():
                for callback in self.callbacks[msg.packet.port]:
                    callback(msg.packet)

    def _prepare_send_request(self) -> CrtpPacketSend.Request:
        req = CrtpPacketSend.Request()
        req.channel = self.channel
        req.address = self.address
        req.datarate = self.datarate
        return req

    def _send_packet(
        self,
        packet: CrtpPacket,
        expects_response: bool = False,
        matching_bytes: int = 0,
    ) -> Future:
        req = self._prepare_send_request()
        req.expects_response = expects_response
        req.matching_bytes = matching_bytes
        req.packet = packet
        return self.send_packet_service.call_async(req)

    def _crtp_link_end_callback(self, msg: CrtpLinkEnd):
        address = msg.address
        if (address == self.address).all():
            self.node.get_logger().info("Address matches, killing us!")
            if self.link_end_callback is not None:
                self.link_end_callback()

    # Override
    def send_packet_no_response(
        self,
        packet: CrtpPacket,
        expects_response: bool = False,
        matching_bytes: int = 0,
    ) -> Future:
        """
        async send packet without waiting for response
        """
        return self._send_packet(packet, expects_response, matching_bytes)

    # Override
    def send_packet(
        self, packet: CrtpPacket, expects_response: bool, matching_bytes: int
    ):
        fut = self._send_packet(packet, expects_response, matching_bytes)
        rclpy.spin_until_future_complete(self.node, fut, executor=self.node.executor)
        if fut.result() is None:
            raise Exception(f"Link did not respond, shutting down {packet}")
        return fut.result().packet

    # Override
    def send_batch_request(self, packets: List[CrtpPacket]):
        """
        Sends out multiple requests at once and waits for all of them.
        For the crazyradio this is more performant then sending them individually because
        concurrent packages help to push answers from crazyflie.
        """
        futures: List[Future] = []
        for pkt in packets:
            packet, expects_response, matching_bytes = pkt
            futures.append(self._send_packet(packet, expects_response, matching_bytes))

        responses = []
        for fut in futures:
            rclpy.spin_until_future_complete(
                self.node, fut, executor=self.node.executor
            )
            responses.append(fut.result())

        return responses
