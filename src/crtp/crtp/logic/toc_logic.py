import struct 
from .toc.toc import Toc
from .toc.toccache import TocCache

class TocLogic:

    def __init__(self, CrtpPacker, CrtpLink, ElementType, path):
        self.ElementType = ElementType
        self.packer = CrtpPacker
        self.link = CrtpLink
        

        self.toc = Toc()
        self.toc_cache = TocCache(rw_cache=path)

        self.nbr_of_items = None
        self.crc = None
    
    def _to_toc_item(self, data):
        ident = struct.unpack('<H', data[1:3])[0]
        data = bytearray(data[3:])
        element = self.ElementType(ident, data)
        self.toc.add_element(element)
    
    def initialize_toc(self):
        nbr_of_items, crc = self.get_toc_info()
        cache_data = self.toc_cache.fetch(crc)
        if (cache_data):
            self.toc.toc = cache_data
        else:
            self.download_toc_items()
    
    def get_toc_info(self):
        packet, expects_response, matching_bytes = self.packer.get_toc_info()
        resp_data = self.link.send_packet(packet, expects_response, matching_bytes).data
        [self.nbr_of_items, self.crc] = struct.unpack('<HI', resp_data[1:7])
        return self.nbr_of_items, self.crc

    def download_toc_items(self):
        if self.nbr_of_items == None or self.crc == None: 
            self.get_toc_info()      
        packets = []
        for i in range(self.nbr_of_items):
            packets.append(self.packer.get_toc_item(i))
        
        responses = self.link.send_batch_request(packets)
        for result in responses: 
            self._to_toc_item(result.packet.data[:result.packet.data_length])

        self.toc_cache.insert(self.crc, self.toc.toc)