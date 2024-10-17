from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import SetGroupMask, Takeoff, Land, Stop, GoTo, StartTrajectory, UploadTrajectory # HL_Commander
from crazyflie_hardware.crtp_driver.crtp_packer_ros import CrtpPackerRos

from crtp.crtp.logic.hl_commander_logic import HighLevelCommanderLogic

class HighLevelCommander(HighLevelCommanderLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPackerRos, CrtpLink)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(SetGroupMask, "~/set_group_mask", self._set_group_mask, 10, callback_group=callback_group)
        node.create_subscription(Takeoff, "~/takeoff", self._takeoff, 10, callback_group=callback_group)
        node.create_subscription(Land, "~/land", self._land, 10, callback_group=callback_group)
        node.create_subscription(GoTo, "~/go_to", self._go_to, 10, callback_group=callback_group)

    def _set_group_mask(self, msg):
        self.set_group_mask(msg.group_mask)
    
    def _go_to(self, msg):
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.go_to(msg.goal.x, msg.goal.y, msg.goal.z, msg.yaw, duration, msg.relative, msg.group_mask)
        
    def _takeoff(self, msg):
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.takeoff(msg.height, duration, msg.group_mask, msg.yaw) 
        
    def _land(self, msg):
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.land(msg.height, duration, msg.group_mask, msg.yaw)
        
    

