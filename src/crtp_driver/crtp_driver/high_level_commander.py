

from crazyflie_interface.msg import SetGroupMask, Takeoff, Land, Stop, GoTo, StartTrajectory, UploadTrajectory # HL_Commander
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crtplib.packers.hl_commander import HighLevelCommanderPacker
from crtp_driver.crtp_packer import CrtpPacker

class HighLevelCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = HighLevelCommanderPacker(CrtpPacker)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(SetGroupMask, "~/set_group_mask", self.set_group_mask, 10, callback_group=callback_group)
        node.create_subscription(Takeoff, "~/takeoff", self.takeoff, 10, callback_group=callback_group)
        node.create_subscription(Land, "~/land", self.land, 10, callback_group=callback_group)
        node.create_subscription(GoTo, "~/go_to", self.go_to, 10, callback_group=callback_group)

    def set_group_mask(self, msg):
        packet = self.packer.set_group_mask(msg.group_mask)
        self.send_crtp_async(packet)
    
    def takeoff(self, msg):
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        packet = self.packer.takeoff(msg.height, duration, msg.group_mask, msg.yaw) 
        self.send_crtp_async(packet)
    
    def land(self, msg):
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        packet = self.packer.land(msg.height, duration, msg.group_mask, msg.yaw)
        self.send_crtp_async(packet)

    def go_to(self, msg):
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        packet = self.packer.go_to(msg.goal.x, msg.goal.y, msg.goal.z, msg.yaw, duration, msg.relative, msg.group_mask)
        self.send_crtp_async(packet)


