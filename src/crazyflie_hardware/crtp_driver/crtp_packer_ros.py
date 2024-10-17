import numpy as np
from crtp_interfaces.msg import CrtpPacket

class CrtpPackerRos:
    def __init__(self, port):
        self.port = port

    def _struct_to_data(self, data):
        """the data field must be an array of size 31"""
        np_array =  np.array([p for p in data], dtype=np.uint8)
        return np.pad(np_array, (0, len(CrtpPacket().data) - len(data)), mode='constant')

    def _fill_packet_data(self, pk, data):
        pk.data = self._struct_to_data(data)
        pk.data_length = len(data)   

    def prepare_packet(self, channel, data) -> CrtpPacket:
        pk = CrtpPacket()
        pk.port = self.port
        pk.channel = channel
        self._fill_packet_data(pk, data)
        pk.data_length = len(data)
        
        return pk  