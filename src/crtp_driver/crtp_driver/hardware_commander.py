import struct

from crtp_driver.crtp_packer import CrtpPacker
from crtplib.packers.link_layer import LinkLayerPacker

from std_msgs.msg import Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HardwareCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = LinkLayerPacker(CrtpPacker)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(Int16, "~/platform_power_down", self.platform_power_down, 10,callback_group=callback_group)
        node.create_subscription(Int16, "~/reboot_to_fw", self.reboot_to_fw, 10,callback_group=callback_group)
        

    def platform_power_down(self, msg):
        packet = self.packer.platform_power_down()
        self.send_crtp_async(packet)

    def reboot_to_fw(self, msg):
        packet = self.packer.reset_init()
        self.send_crtp_async(packet)
        
        packet = self.packer.reset_to_bootloader()
        self.send_crtp_async(packet)

