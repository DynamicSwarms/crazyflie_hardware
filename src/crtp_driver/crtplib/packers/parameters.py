import struct

from .packer import Packer

class ParameterCommanderPacker(Packer):
    PORT_PARAMETER = 0x02
    
    TOC_CHANNEL = 0
    READ_CHANNEL = 1
    WRITE_CHANNEL = 2
    MISC_CHANNEL = 3

    CMD_TOC_ITEM_V2 = 2 
    CMD_TOC_INFO_V2 = 3 


    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_PARAMETER)
        #node.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  500)
        #self.node = node
        #self.count = 0

        #self.state = IDLE
               
    # Overwrite
    def _prepare_packet(self, channel, data):
        return super()._prepare_packet(channel=channel, data=data)
 
    def get_loc_info(self):
        data = struct.pack('<B',
                           self.CMD_TOC_INFO_V2)                           
        packet = self._prepare_packet(self.TOC_CHANNEL, data)
        return packet, True, 1 
    
    def get_toc_item(self, index):
        data = struct.pack('<BBB',
                           self.CMD_TOC_ITEM_V2, 
                           index & 0xFF,
                           (index >> 8) & 0xFF )
        return self._prepare_packet(self.TOC_CHANNEL, data), True, 3   
     
    
    def set_parameter(self, id, pytype, value):
        data = struct.pack('<H', id)
        data += struct.pack(pytype, value)
        return self._prepare_packet(self.WRITE_CHANNEL, data)
        

