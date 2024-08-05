import struct 

from .packer import Packer

class LoggingPacker(Packer):
    PORT_LOGGING = 5

    # Channels used for the logging port
    TOC_CHANNEL = 0
    CONTROL_CHANNEL = 1
    LOGDATA_CHANNEL = 2

    # Commands used when accessing the Table of Contents
    CMD_TOC_ELEMENT = 0  # original version: up to 255 entries
    CMD_TOC_INFO = 1    # original version: up to 255 entries
    CMD_GET_ITEM_V2 = 2  # version 2: up to 16k entries
    CMD_GET_INFO_V2 = 3  # version 2: up to 16k entries

    # Commands used when accessing the Log configurations
    CMD_CREATE_BLOCK = 0
    CMD_APPEND_BLOCK = 1
    CMD_DELETE_BLOCK = 2
    CMD_START_LOGGING = 3
    CMD_STOP_LOGGING = 4
    CMD_RESET_LOGGING = 5
    CMD_CREATE_BLOCK_V2 = 6
    CMD_APPEND_BLOCK_V2 = 7

    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_LOGGING)

    def get_toc_info(self):
        data = struct.pack('<B', 
                           self.CMD_GET_INFO_V2)
        return self._prepare_packet(channel=self.TOC_CHANNEL, data=data), True, 1
    
    def get_toc_item(self, index):
        data = struct.pack('<BBB',
                           self.CMD_GET_ITEM_V2, 
                           index & 0xFF,
                           (index >> 8) & 0xFF )
        return self._prepare_packet(self.TOC_CHANNEL, data), True, 3 
    
    def create_block_content(self, content):
        data = struct.pack('<')
        for el in content: 
            storage_and_fetch, index = el
            data += struct.pack('<BBB', 
                                storage_and_fetch,
                                index & 0xFF,
                                (index >> 8) & 0xFF 
                                ) # storage and fetch byte
        return data
            

    def create_block(self, index, content):
        data = struct.pack('<BB', self.CMD_CREATE_BLOCK_V2, index)
        data += self.create_block_content(content)
        return self._prepare_packet(self.CONTROL_CHANNEL, data), True, 2
    
    def start_block(self, index, period):
        data = struct.pack('<BBB',
                           self.CMD_START_LOGGING,
                           index, 
                           period)
        return self._prepare_packet(self.CONTROL_CHANNEL, data), True, 2
    
    def stop_block(self, index):
        data = struct.pack('<BB',
                           self.CMD_STOP_LOGGING,
                           index)
        return self._prepare_packet(self.CONTROL_CHANNEL, data), True, 2