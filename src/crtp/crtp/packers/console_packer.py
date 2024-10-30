import struct 

from .packer import Packer

class ConsolePacker(Packer):
    PORT_CONSOLE = 0

    CHANNEL_CONSOLE = 0

    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_CONSOLE)

    def consolepacket(self):
        data = struct.pack('<')
        return self._prepare_packet(channel=self.CHANNEL_CONSOLE,
                                    data=data)