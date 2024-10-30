import struct
from .packer import Packer

class BasicCommanderPacker(Packer):
    PORT_BASIC_COMMANDER = 0x03

    CHANNEL = 0x0

    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_BASIC_COMMANDER)    

    def send_setpoint(self, roll, pitch, yawrate, thrust):
        data = struct.pack('<fffH',
                           roll, 
                           pitch, 
                           yawrate, 
                           thrust)
        return self._prepare_packet(data=data)
