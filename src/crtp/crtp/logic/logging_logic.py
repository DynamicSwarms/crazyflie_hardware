from crtp.packers.logging_packer import LoggingPacker

from .toc.logging import LogTocElement
from .toc_logic import TocLogic

class LoggingLogic(TocLogic):

    def __init__(self, CrtpPacker, CrtpLink, path):
        self.link = CrtpLink
        self.packer = LoggingPacker(CrtpPacker)
        super().__init__(self.packer, self.link, LogTocElement, path)  

    def start_block(self, id, period):
        packet, expects_response, matching = self.packer.start_block(id, period)
        self.link.send_packet_no_response(packet)
        
    def add_block(self, id, elements):
        stX = self.toc.get_element_by_complete_name("stateEstimate.x")
        stY = self.toc.get_element_by_complete_name("stateEstimate.y")
        stZ = self.toc.get_element_by_complete_name("stateEstimate.z")
        typeX = LogTocElement.get_id_from_cstring(stX.ctype)
        typeY = LogTocElement.get_id_from_cstring(stY.ctype)
        typeZ = LogTocElement.get_id_from_cstring(stZ.ctype)
        elements = [(typeX, stX.ident),(typeY, stY.ident), (typeZ, stZ.ident) ]
        packet, expects_response, matching = self.packer.create_block(id, elements)
        self.link.send_packet_no_response(packet)

