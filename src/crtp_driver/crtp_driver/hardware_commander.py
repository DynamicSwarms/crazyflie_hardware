import struct

from crtp_driver.crtp_packer import CrtpPacker
from crtplib.logic.link_layer_logic import LinkLayerLogic

from std_msgs.msg import Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HardwareCommander(LinkLayerLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPacker, CrtpLink)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(Int16, "~/platform_power_down", self._platform_power_down, 10,callback_group=callback_group)
        node.create_subscription(Int16, "~/reboot_to_firmware", self._reboot_to_firmware, 10,callback_group=callback_group)
        node.create_subscription(Int16, "~/reboot_to_bootloader", self._reboot_to_bootloader, 10,callback_group=callback_group)
        

    def _platform_power_down(self, msg):
        self.platform_power_down()

    def _reboot_to_firmware(self, msg):
        self.reboot_to_firmware()

    def _reboot_to_bootloader(self, msg):
        self.reboot_to_bootloader()
