
from crtp_interface.msg import CrtpResponse
import struct 
import time

from .toccache import TocCache
from .param import ParamTocElement, Toc
IDLE = 0
REQ_INFO = 1
REQ_ITEM = 2
CMD_TOC_ITEM_V2 = 2 
CMD_TOC_INFO_V2 = 3 

import os


class ParamReader():

    def __init__(self, node):
        node.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  500)
        self.node = node
        self.count = 0

        self.state = IDLE
        
        self.toc = Toc()

        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "param")
        self.toc_cache = TocCache(rw_cache=path)
 
    def get_loc_toc(self):
        #self.node.get_logger().info(str(os.listdir(os.environ.get("$HOME")))  +" "+ str(os.getcwd()))
        #self.node.get_logger().info(str(os.environ.get("HOME")))
        #return
        if self.state == REQ_ITEM:
            self.node.get_logger().info(str(len(self.params)))
            self.node.get_logger().info(str(self.params))
            self.state = IDLE
            return
        self.params = []
        req = self.node._prepare_send_request()

#        req.packet.port = 2 # log
#        req.packet.channel = 0 # access
#        req.packet.data[0] = 0x00 # reset toc pointer
#        req.data_length = 1
#        self.send_packet_service.call_async(req) # send the toc pointer reset
       

        req.packet.port = 2 # log
        req.packet.channel = 0 # access
        req.packet.data[0] = CMD_TOC_INFO_V2 # v2#0x01 # assuming this is message id then this is "get next toc element"
        req.packet.data_length = 2

        self.state = REQ_INFO
        self.node.send_packet_service.call_async(req)
        self.node.send_null_packet("")
        self.node.send_null_packet("")

        #req.packet.data[0] = CMD_TOC_ITEM_V2
        #req.packet.data[1] = 1
        #self.send_packet_service.call_async(req)
#
        #self.param_reader.set_count(msg.data)
    
    def handle_response(self, response):
        data = response.packet.data
        data_length = response.packet.data_length
        channel = response.packet.channel
        port = response.packet.port

        if port != 2 or channel != 0 or not data_length: return # not us 
        #self.node.get_logger().info("Received in ParamReader" + str(self.state) +str(data[0]) )
        #self.node.get_logger().info(str(response.packet))
        if self.state == REQ_INFO and data[0] == CMD_TOC_INFO_V2:
            [self.nbr_of_items, self._crc] = struct.unpack('<HI', data[1:7])
            self.node.get_logger().info(str("NBR of Items: "+ str(self.nbr_of_items)))
            #self.node.get_logger().info(str(self._crc))

            #self.count = 0
            #self.get_next()

            if False: # self.toc_cache.fetch(self._crc) is not None:
                self.state = IDLE
            else:   # retrieve
                for i in range(self.nbr_of_items):
                    self.get_idx(i)
                
                self.node.send_null_packet("")
                self.node.send_null_packet("")

                self.state = REQ_ITEM

            #self.node.send_null_packet("")
            #self.node.send_null_packet("")
        elif self.state == REQ_ITEM and data[0] == CMD_TOC_ITEM_V2:
            #data: cmd, ident, group, name
            #self.node.get_logger().info(str(data))
            ident = struct.unpack('<H', data[1:3])[0]
            data_ = bytearray(data[3:])
            element = ParamTocElement(ident, data_)

            self.node.get_logger().info("New Element: '" + str(element.ident) + "' '" + element.group + "' '" + element.name + "'")


            self.params.append(element)
            self.toc.add_element(element)

            self.node.get_logger().info("Recv: " + str(len(self.params)))
            if len(self.params) == self.nbr_of_items:
                self.state = IDLE
                
                self.toc_cache.insert(self._crc,self.toc.toc )
                self.node.get_logger().info("Received all Params, Writing to cache")
                #for param in self.params:
                #    self.node.get_logger().info(str(param.ident) + ": "  + param.group + " " + param.name)
        else:
            pass
        
       
    def get_idx(self, idx):
        req = self.node._prepare_send_request()
        req.packet.port = 2 # log
        req.packet.channel = 0 # access

        req.packet.data_length = 3
        req.packet.data[0] = CMD_TOC_ITEM_V2
        req.packet.data[1] = idx & 0x0ff
        req.packet.data[2] = (idx >> 8) & 0x0ff
        req.expects_response = True
        req.matching_bytes = 3 # Watchdog shall Guard these messages
        self.node.send_packet_service.call_async(req)
        #self.node.send_null_packet("")
        #self.node.send_null_packet("")
        self.node.get_logger().info(str("requesting" + str(idx)))
     
