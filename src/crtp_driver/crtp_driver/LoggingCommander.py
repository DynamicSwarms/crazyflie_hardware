import struct 
import os

import rclpy

from .toccache import TocCache
from .log import LogTocElement
from .toc import Toc
from crtp_driver.crtp_packer import CrtpPacker

from std_msgs.msg import Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crtplib.packers.logging import LoggingPacker

class LoggingCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async
        self.node = node

        self.toc = Toc()

        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "log")
        self.toc_cache = TocCache(rw_cache=path)

        self.packer = LoggingPacker(CrtpPacker)

        callback_group = MutuallyExclusiveCallbackGroup()
        node.create_subscription(Int16, "~/get_logging_toc", self.download_toc, 10, callback_group=callback_group)

    def load_toc(self):
        packet, expects_response, matching_bytes = self.packer.get_toc_info()        
        resp_packet = self.send_crtp_sync(packet, expects_response, matching_bytes)
        data = resp_packet.data

        [self.nbr_of_items, self._crc] = struct.unpack('<HI', data[1:7])
        cache_data = self.toc_cache.fetch(self._crc)
        if (cache_data): 
            self.node.get_logger().info(str("Loaded toc from cache"))
            self.toc.toc = cache_data
            #for group in cache_data:
            #     for name in cache_data[group]:
            #        self.node.declare_parameter(str(group) + "." + str(name), rclpy.Parameter.Type.DOUBLE)     
        else:
            self.download_toc()  

    def download_toc(self, msg=None):
        self.log_items = []

        packet, expects_response, matching_bytes = self.packer.get_toc_info()
        
        resp_packet = self.send_crtp_sync(packet, expects_response, matching_bytes)
        data = resp_packet.data

        
        [self.nbr_of_items, self._crc] = struct.unpack('<HI', data[1:7])
        self.node.get_logger().info(str("NBR of Items: "+ str(self.nbr_of_items)))

        futures = []
        for i in range(self.nbr_of_items):
            ret = self.get_toc_item(i) 
            futures.append(ret)

        responses = []
        for fut in futures:
            rclpy.spin_until_future_complete(self.node, fut)
            result =  fut.result()
            responses.append(result)

        for result in responses: 
            self.to_log_item(result.packet.data[:result.packet.data_length])

    def get_toc_item(self, index):
        packet, expects_response, response_bytes = self.packer.get_toc_item(index)
        self.node.get_logger().info(str("Requesting" + str(index)))
        return self.send_crtp_async(packet, expects_response,response_bytes)

    def to_log_item(self, data):
        ident = struct.unpack('<H', data[1:3])[0]
        data_ = bytearray(data[3:])
        element = LogTocElement(ident, data_)

        self.node.get_logger().info("New Element: '" + str(element.ident) + "' '" + element.group + "' '" + element.name + "'")

        self.log_items.append(element)
        self.toc.add_element(element)

        self.node.get_logger().info("Recv: " + str(len(self.log_items)))
        if len(self.log_items) == self.nbr_of_items:
            self.toc_cache.insert(self._crc,self.toc.toc )
            self.node.get_logger().info("Received all Params, Writing to cache")

    def start_block(self, id, period):
        packet, expects_response, matching = self.packer.start_block(id, period)
        self.send_crtp_async(packet)
        
    def add_block(self, id, elements):
        stX = self.toc.get_element_by_complete_name("stateEstimate.x")
        stY = self.toc.get_element_by_complete_name("stateEstimate.y")
        stZ = self.toc.get_element_by_complete_name("stateEstimate.z")
        typeX = LogTocElement.get_id_from_cstring(stX.ctype)
        typeY = LogTocElement.get_id_from_cstring(stY.ctype)
        typeZ = LogTocElement.get_id_from_cstring(stZ.ctype)
        elements = [(typeX, stX.ident),(typeY, stY.ident), (typeZ, stZ.ident) ]
        packet, expects_response, matching = self.packer.create_block(id, elements)
        self.send_crtp_async(packet)

