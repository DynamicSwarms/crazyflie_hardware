import rclpy

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from crtp_interfaces.srv import CrtpPacketSend
from crtp_interfaces.msg import CrtpLinkEnd

class CrtpLinkRos:
    def __init__(self, node, channel, address, datarate, link_end_callback):
        self.channel = channel
        self.address = address
        self.datarate = datarate
        self.link_end_callback = link_end_callback 
        self.node = node


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.send_packet_service = node.create_client(CrtpPacketSend, "/crazyradio/send_crtp_packet",
                                                      qos_profile=qos_profile)
        while not self.send_packet_service.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("Send CRTP Packet Service not available, waiting again...")

        node.create_subscription(CrtpLinkEnd, "/crazyradio/crtp_link_end", self._crtp_link_end_callback, 10)

    def _prepare_send_request(self):
        req = CrtpPacketSend.Request()
        req.channel = self.channel
        req.address = self.address
        req.datarate = self.datarate
        return req
    
    def _send_packet(self, packet, expects_response=False, matching_bytes=0):
        req = self._prepare_send_request()
        req.expects_response = expects_response
        req.matching_bytes = matching_bytes
        req.packet = packet
        return self.send_packet_service.call_async(req)

    def _crtp_link_end_callback(self, msg):
        address = msg.address
        if (address == self.address).all():
            self.node.get_logger().info("Address mathces, killing us")
            if self.link_end_callback is not None: self.link_end_callback()

    def send_packet_no_response(self, packet, expects_response=False, matching_bytes=0):
        '''
        async send packet without waiting for response
        '''
        return self._send_packet(packet, expects_response, matching_bytes)

    def send_packet(self, packet, expects_response, matching_bytes):
        fut = self._send_packet(packet, expects_response, matching_bytes)
        rclpy.spin_until_future_complete(self.node, fut)
        return fut.result().packet

    def send_batch_request(self, packets):
        futures = []
        for pkt in packets:
            packet, expects_response, matching_bytes = pkt
            futures.append(self._send_packet(packet, expects_response, matching_bytes))

        responses = []
        for fut in futures:
            rclpy.spin_until_future_complete(self.node, fut)
            responses.append(fut.result())

        return responses