class Packer:

    def __init__(self, CrtpPacker, port):
        self.packer = CrtpPacker(port)

    def _prepare_packet(self, channel, data):
        return self.packer.prepare_packet(channel=channel, data=data)
