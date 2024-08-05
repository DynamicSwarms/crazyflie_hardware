import struct

from .toc_packer import TocPacker

class ParametersPacker(TocPacker):
    PORT_PARAMETER = 0x02
    
    TOC_CHANNEL = 0
    READ_CHANNEL = 1
    WRITE_CHANNEL = 2
    MISC_CHANNEL = 3


    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_PARAMETER)
    
    def set_parameter(self, id, pytype, value):
        data = struct.pack('<H', id)
        data += struct.pack(pytype, value)
        return self._prepare_packet(self.WRITE_CHANNEL, data)
        

