from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from crtp_interface.srv import CrtpPacketSend
import rclpy


class CrtpLinkRos:
    def __init__(self, node, channel, address, datarate):
        self.channel = channel
        self.address = address
        self.datarate = datarate
        self.node = node


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.send_packet_service = node.create_client(CrtpPacketSend, "/crazyradio/send_crtp_packet",
                                                      qos_profile=qos_profile)
        while not self.send_packet_service.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("Send CRTP Packet Service not available, waiting again...")

    def _prepare_send_request(self):
        req = CrtpPacketSend.Request()
        req.channel = self.channel
        req.address = self.address
        req.datarate = self.datarate
        return req

    def send_packet_no_response(self, packet, expects_response=False, matching_bytes=0):
        '''
        async send packet without waiting for response
        '''
        req = self._prepare_send_request()
        req.expects_response = expects_response
        req.matching_bytes = matching_bytes
        req.packet = packet
        return self.send_packet_service.call_async(req)

    def send_packet(self, packet, expects_response, matching_bytes):
        fut = self.send_crtp_packet_async(packet, expects_response, matching_bytes)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().packet

    def send_batch_request(self, packets):
        futures = []
        for pkt in packets:
            packet, expects_response, matching_bytes = pkt
            futures.append(self.send_crtp_packet_async(packet, expects_response, matching_bytes))

        responses = []
        for fut in futures:
            rclpy.spin_until_future_complete(self, fut)
            responses.append(fut.result())

        return responses