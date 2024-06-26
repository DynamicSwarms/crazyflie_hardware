
from crtp_interface.msg import CrtpResponse
import struct 
IDLE = 0
REQ_INFO = 1
REQ_ITEM = 2
CMD_TOC_ITEM_V2 = 2 
CMD_TOC_INFO_V2 = 3 


class ParamReader():

    def __init__(self, node):
        node.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  500)
        self.node = node
        self.count = 0

        self.state = IDLE

    def get_loc_toc(self):
        if self.state == REQ_ITEM:
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


            for i in range(self.nbr_of_items):
                self.get_idx(i)
            self.state = REQ_ITEM

            #self.node.send_null_packet("")
            #self.node.send_null_packet("")
        elif self.state == REQ_ITEM and data[0] == CMD_TOC_ITEM_V2:
            name = data[4:data_length]
            string = "New log_toc entry: '"
            string = string + " " + str(data[0]) + " " +str(data[1]) +  " " +str(data[2]) + " " + str(data[3]) + " "
            str_name = ""

            if len(name): 
                for ch in name[:-1]:
                    if ch == 0: str_name = str_name + " " 
                    else: str_name = str_name + chr(ch)
                str_name = str_name + "'"

                self.params.append(str_name)
            self.node.get_logger().info(str(string + " " + str(str_name)))

            #if (self.count < self.nbr_of_items): 
            #    self.get_next()
            #else: 
            #    self.state = IDLE
            #    self.node.get_logger().info(str(self.params))
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
        self.node.send_packet_service.call_async(req)
        self.node.send_null_packet("")
        self.node.send_null_packet("")
        #self.node.get_logger().info(str("requesting" + str(idx)))
     
