from crtplib.packers.link_layer import LinkLayerPacker

class LinkLayerLogic:

    def __init__(self, CrtpPacker, CrtpLink):
        self.link = CrtpLink
        self.packer = LinkLayerPacker(CrtpPacker)
    
    def platform_power_down(self):
        packet = self.packer.platform_power_down()
        self.link.send_packet_no_response(packet)

    
    def reboot_to_bootloader(self):
        packet = self.packer.reset_init()
        self.link.send_packet_no_response(packet)

        packet = self.packer.reset_to_bootloader()
        self.link.send_packet_no_response(packet)

    def reboot_to_firmware(self):
        packet = self.packer.reset_init()
        self.link.send_packet_no_response(packet)

        packet = self.packer.reset_to_firmware()
        self.link.send_packet_no_response(packet)

