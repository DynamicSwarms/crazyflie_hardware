from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crtp_driver.crtp_packer import CrtpPacker
from crtplib.logic.basic_commander_logic import BasicCommanderLogic

class BasicCommander(BasicCommanderLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPacker, CrtpLink)

        callback_group = MutuallyExclusiveCallbackGroup()

        # TODO: sendSetpoint not implemented yet
        #node.create_subscription(SendSetpoint, "~/send_setpoint", self._send_setpoint, 10,callback_group=callback_group) 

    def _send_setpoint(self, msg):
        self.send_setpoint(msg.roll, msg.pitch, msg.yawrate, msg.thrust)
        
