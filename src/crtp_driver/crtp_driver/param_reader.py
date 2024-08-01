
from crtp_interface.msg import CrtpResponse
import struct 
import time
import rclpy

from .toccache import TocCache
from .param import ParamTocElement, Toc
from crtp_driver.crtp_packer import CrtpPacker

from std_msgs.msg import Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

IDLE = 0
REQ_INFO = 1
REQ_ITEM = 2
CMD_TOC_ITEM_V2 = 2 
CMD_TOC_INFO_V2 = 3 

import os

class ParameterCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async
        self.node = node

        self.toc = Toc()

        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "param")
        self.toc_cache = TocCache(rw_cache=path)

        self.packer = ParameterCommanderPacker()

        callback_group = MutuallyExclusiveCallbackGroup()
        node.create_subscription(Int16, "~/get_loc_toc", self.get_loc_toc, 10, callback_group=callback_group)



    def get_loc_or_load(self):
        packet, expects_response, matching_bytes = self.packer.get_loc_info()        
        resp_packet = self.send_crtp_sync(packet, expects_response, matching_bytes)
        data = resp_packet.data

        [self.nbr_of_items, self._crc] = struct.unpack('<HI', data[1:7])
        cache_data = self.toc_cache.fetch(self._crc)
        if (cache_data): 
            self.node.get_logger().info(str("Loaded toc from cache"))
            self.toc.toc = cache_data
            for group in cache_data:
                 for name in cache_data[group]:
                    self.node.declare_parameter(str(group) + "." + str(name), rclpy.Parameter.Type.DOUBLE)
                    
        else:
            self.get_loc_toc()
    



    def get_loc_toc(self, msg):
        self.params = []

        packet, expects_response, matching_bytes = self.packer.get_loc_info()
        
        resp_packet = self.send_crtp_packet_sync(packet, expects_response, matching_bytes)
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
            self.to_loc_item(result.packet.data)

    def get_toc_item(self, index):
        packet, expects_response, response_bytes = self.packer.get_toc_item(index)
        self.node.get_logger().info(str("Requesting" + str(index)))
        return self.send_crtp_async(packet, expects_response,response_bytes)
    def set_parameters(self, par_dict):
        #
        pass

    def set_parameter(self, group, name, value):
        toc_element = self.toc.get_element(group, name) ## Error checking!!
        id = toc_element.ident
        if toc_element.pytype == '<f' or toc_element.pytype == '<d':
            value_nr = float(value)
        else:
            value_nr = int(value)
        
        packet = self.packer.set_parameter(id, toc_element.pytype, value_nr)
        self.send_crtp_async(packet)

    def to_loc_item(self, data):
        ident = struct.unpack('<H', data[1:3])[0]
        data_ = bytearray(data[3:])
        element = ParamTocElement(ident, data_)

        self.node.get_logger().info("New Element: '" + str(element.ident) + "' '" + element.group + "' '" + element.name + "'")

        self.params.append(element)
        self.toc.add_element(element)

        self.node.get_logger().info("Recv: " + str(len(self.params)))
        if len(self.params) == self.nbr_of_items:
            self.toc_cache.insert(self._crc,self.toc.toc )
            self.node.get_logger().info("Received all Params, Writing to cache")


class ParameterCommanderPacker(CrtpPacker):
    PORT_PARAMETER = 0x02
    
    TOC_CHANNEL = 0
    READ_CHANNEL = 1
    WRITE_CHANNEL = 2
    MISC_CHANNEL = 3

    def __init__(self):
        super().__init__(self.PORT_PARAMETER)
        #node.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  500)
        #self.node = node
        #self.count = 0

        #self.state = IDLE
               
    # Overwrite
    def _prepare_packet(self, channel, data):
        return super()._prepare_packet(channel=channel, data=data)
 
    def get_loc_info(self):
        data = struct.pack('<B',
                           CMD_TOC_INFO_V2)                           
        packet = self._prepare_packet(self.TOC_CHANNEL, data)
        return packet, True, 1 
    
    def get_toc_item(self, index):
        data = struct.pack('<BBB',
                           CMD_TOC_ITEM_V2, 
                           index & 0xFF,
                           (index >> 8) & 0xFF )
        return self._prepare_packet(self.TOC_CHANNEL, data), True, 3   
     
    
    def set_parameter(self, id, pytype, value):
        data = struct.pack('<H', id)
        data += struct.pack(pytype, value)
        return self._prepare_packet(self.WRITE_CHANNEL, data)
        

