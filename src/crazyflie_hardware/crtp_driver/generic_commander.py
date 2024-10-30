from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from crazyflie_interfaces.msg import NotifySetpointsStop, VelocityWorld, Hover, FullState, Position # (Generic)Commander

from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.generic_commander_logic import GenericCommanderLogic

class GenericCommander(GenericCommanderLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPackerRos, CrtpLink)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(NotifySetpointsStop, "~/notify_setpoints_stop", self._notify_setpoints_stop, 10,callback_group=callback_group) 
        node.create_subscription(VelocityWorld, "~/cmd_vel", self._cmd_vel, 10, callback_group=callback_group)
        node.create_subscription(Hover, "~/cmd_hover", self._cmd_hover, 10,callback_group=callback_group)
        node.create_subscription(FullState, "~/cmd_full_state", self._cmd_full_state, 10, callback_group=callback_group)
        node.create_subscription(Position, "~/cmd_position", self._cmd_position, 10, callback_group=callback_group)

    def _notify_setpoints_stop(self, msg):
        #TODO: check if group mask might be used
        self.send_notify_setpoints_stop(msg.remain_valid_millisecs)

    def _cmd_vel(self, msg):
        self.send_velocity_world_setpoint(msg.vel.x, msg.vel.y, msg.vel.z, msg.yaw_rate)
        
    def _cmd_hover(self, msg):
        self.send_hover_setpoint(msg.vx, msg.vy, msg.yawrate, msg.z_distance)
        
    def _cmd_full_state(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        acc = [msg.acc.x, msg.acc.y, msg.acc.z]
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w ]
        rollRate = msg.twist.angular.x
        pitchRate = msg.twist.angular.y
        yawRate = msg.twist.angular.z
        self.send_full_state_setpoint(pos, vel, acc, orientation, rollRate, pitchRate, yawRate)
        
    def _cmd_position(self, msg):
        self.send_position_setpoint(msg.x, msg.y, msg.z, msg.yaw)
        
