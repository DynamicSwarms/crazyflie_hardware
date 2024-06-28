
from crtp_interface.msg import CrtpResponse
import struct 
import time
IDLE = 0
REQ_INFO = 1
REQ_ITEM = 2
CMD_TOC_ITEM_V2 = 2 
CMD_TOC_INFO_V2 = 3 


class ParamTocElement:
    """An element in the Log TOC."""

    RW_ACCESS = 0
    RO_ACCESS = 1

    EXTENDED_PERSISTENT = 1

    types = {0x08: ('uint8_t', '<B'),
             0x09: ('uint16_t', '<H'),
             0x0A: ('uint32_t', '<L'),
             0x0B: ('uint64_t', '<Q'),
             0x00: ('int8_t', '<b'),
             0x01: ('int16_t', '<h'),
             0x02: ('int32_t', '<i'),
             0x03: ('int64_t', '<q'),
             0x05: ('FP16', ''),
             0x06: ('float', '<f'),
             0x07: ('double', '<d')}

    def __init__(self, ident=0, data=None):
        """TocElement creator. Data is the binary payload of the element."""
        self.ident = ident
        self.persistent = False
        self.extended = False
        if (data):
            strs = struct.unpack('s' * len(data[1:]), data[1:])
            s = ''
            for ch in strs:
                s += ch.decode('ISO-8859-1')
            strs = s.split('\x00')
            self.group = strs[0]
            self.name = strs[1]

            metadata = data[0]
            if isinstance(metadata, str):
                metadata = ord(metadata)

            # If the fouth byte (1 << 4) (0x10) is set we have extended
            # type information for this element.
            self.extended = ((metadata & 0x10) != 0)

            self.ctype = self.types[metadata & 0x0F][0]
            self.pytype = self.types[metadata & 0x0F][1]
            if ((metadata & 0x40) != 0):
                self.access = ParamTocElement.RO_ACCESS
            else:
                self.access = ParamTocElement.RW_ACCESS

    def get_readable_access(self):
        if (self.access == ParamTocElement.RO_ACCESS):
            return 'RO'
        return 'RW'

    def is_extended(self):
        return self.extended

    def mark_persistent(self):
        self.persistent = True

    def is_persistent(self):
        return self.persistent

class ParamReader():

    def __init__(self, node):
        node.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  500)
        self.node = node
        self.count = 0

        self.state = IDLE

    def get_loc_toc(self):
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


            for i in range(self.nbr_of_items):
                self.get_idx(i)
            
            self.node.send_null_packet("")
            self.node.send_null_packet("")

            self.state = REQ_ITEM

            #self.node.send_null_packet("")
            #self.node.send_null_packet("")
        elif self.state == REQ_ITEM and data[0] == CMD_TOC_ITEM_V2:
            #data: cmd, ident, group, name
            self.node.get_logger().info(str(data))
            ident = struct.unpack('<H', data[1:3])[0]
            data_ = bytearray(data[3:])
            element = ParamTocElement(ident, data_)

            self.node.get_logger().info("New Element: '" + element.group + "' '" + element.name + "'")


            self.params.append(element)

            if len(self.params) == self.nbr_of_items:
                for param in self.params:
                    self.node.get_logger().info(str(param.ident) + ": "  + param.group + " " + param.name)
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
        req.response_bytes = 3 # Watchdog shall Guard these messages
        self.node.send_packet_service.call_async(req)
        #self.node.send_null_packet("")
        #self.node.send_null_packet("")
        self.node.get_logger().info(str("requesting" + str(idx)))
     
