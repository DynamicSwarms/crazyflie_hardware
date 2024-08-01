import struct
import numpy as np
from crtp_interface.msg import CrtpPacket

from crtp_driver.crtp_packer import CrtpPacker


from std_msgs.msg import Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HardwareCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = HardwareCommanderPacker()

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

class HardwareCommanderPacker(CrtpPacker):
    BOOTLOADER_HEADER = 0xfe
    BOOTLOADER_CMD_ALLOFF = 0x01
    BOOTLOADER_CMD_SYSOFF = 0x02
    BOOTLOADER_CMD_SYSON = 0x03
    BOOTLOADER_CMD_RESET_INIT = 0xFF
    BOOTLOADER_CMD_RESET = 0xF0

    BOOTLOADER_RESET_TO_BOOTLOADER = 0
    BOOTLOADER_RESET_TO_FIRMWARE = 1

    PORT_LINK = 0xF
    CHANNEL_LINK = 0xF


    def __init__(self):
        super().__init__(self.PORT_LINK)
    
    # Overwrite
    def _prepare_packet(self, data):
        return super()._prepare_packet(channel=self.CHANNEL_LINK, 
                                       data=data)


    def platform_power_down(self):
        data = struct.pack('<BB',
                            self.BOOTLOADER_HEADER,
                            self.BOOTLOADER_CMD_ALLOFF)
        return self._prepare_packet(data)
    
    def stm_power_down(self):
        data = struct.pack('<BB',
                           self.BOOTLOADER_HEADER,
                           self.BOOTLOADER_CMD_SYSOFF)
        return self._prepare_packet(data)
    
    def stm_power_up(self):
        data = struct.pack('<BB',
                           self.BOOTLOADER_HEADER,
                           self.BOOTLOADER_CMD_SYSON)
        return self._prepare_packet(data)
    
    def reset_init(self):
        data = struct.pack('<BB',
                           self.BOOTLOADER_HEADER,
                           self.BOOTLOADER_CMD_RESET_INIT) 
        return self._prepare_packet(data)
    
    def reset_to_bootloader(self):
        """
        Resets the cf to bootloader, has to be called before 
        """
        data = struct.pack('<BBB',
                           self.BOOTLOADER_HEADER,
                           self.BOOTLOADER_CMD_RESET,
                           self.BOOTLOADER_RESET_TO_BOOTLOADER)
        return self._prepare_packet(data)

    def reset_to_firmware(self):
        """
        Resets the cf to bootloader, has to be called before 
        """
        data = struct.pack('<BBB',
                           self.BOOTLOADER_HEADER,
                           self.BOOTLOADER_CMD_RESET,
                           self.BOOTLOADER_RESET_TO_FIRMWARE)
        return self._prepare_packet(data)