from crtp.packers.crtp_packer import CrtpPacker
from crtp.crtp_link import CrtpLink
from crtp.packers.logging_packer import LoggingPacker

from .toc.logging import LogTocElement
from .toc_logic import TocLogic

from typing import Callable


class LoggingLogic(TocLogic):

    def __init__(
        self,
        crtp_packer_factory: Callable[[int], CrtpPacker],
        crtp_link: CrtpLink,
        path: str,
    ):
        self.__packer = LoggingPacker(crtp_packer_factory)
        super().__init__(self.__packer, crtp_link, LogTocElement, path)

    def start_block(self, id, period):
        packet, expects_response, matching = self.__packer.start_block(id, period)
        self.link.send_packet_no_response(packet)

    def add_block(self, id, elements):
        stX = self.toc.get_element_by_complete_name("stateEstimate.x")
        stY = self.toc.get_element_by_complete_name("stateEstimate.y")
        stZ = self.toc.get_element_by_complete_name("stateEstimate.z")
        typeX = LogTocElement.get_id_from_cstring(stX.ctype)
        typeY = LogTocElement.get_id_from_cstring(stY.ctype)
        typeZ = LogTocElement.get_id_from_cstring(stZ.ctype)
        elements = [(typeX, stX.ident), (typeY, stY.ident), (typeZ, stZ.ident)]
        packet, expects_response, matching = self.__packer.create_block(id, elements)
        self.link.send_packet_no_response(packet)
