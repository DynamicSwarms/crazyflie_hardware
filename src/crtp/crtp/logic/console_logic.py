from crtp.packers.console_packer import ConsolePacker

class ConsoleLogic:

    def __init__(self, CrtpPacker, CrtpLink):
        self.link = CrtpLink
        self.packer = ConsolePacker(CrtpPacker)

    def send_consolepacket(self):
        packet = self.packer.consolepacket()
        self.link.send_packet_no_response(packet)