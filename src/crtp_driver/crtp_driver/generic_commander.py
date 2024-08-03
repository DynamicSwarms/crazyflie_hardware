"""
Used for sending control setpoints to the Crazyflie
"""


from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from crazyflie_interface.msg import NotifySetpointsStop, VelocityWorld, Hover, FullState, Position # (Generic)Commander

from crtplib.packers.generic_commander import GenericCommanderPacker
from crtp_driver.crtp_packer import CrtpPacker

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

class GenericCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = GenericCommanderPacker(CrtpPacker)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(NotifySetpointsStop, "~/notify_setpoints_stop", self.notify_setpoints_stop, 10,callback_group=callback_group) 
        node.create_subscription(VelocityWorld, "~/cmd_vel", self.cmd_vel, 10, callback_group=callback_group)
        node.create_subscription(Hover, "~/cmd_hover", self.cmd_hover, 10,callback_group=callback_group)
        node.create_subscription(FullState, "~/cmd_full_state", self.cmd_full_state, 10, callback_group=callback_group)
        node.create_subscription(Position, "~/cmd_position", self.cmd_position, 10, callback_group=callback_group)

    def notify_setpoints_stop(self, msg):
        #TODO: check if group mask might be used
        packet = self.packer.send_notify_setpoint_stop(msg.remain_valid_millisecs)
        self.send_crtp_async(packet)

    def cmd_vel(self, msg):
        packet = self.packer.send_velocity_world_setpoint(msg.vel.x, msg.vel.y, msg.vel.z, msg.yaw_rate)
        self.send_crtp_async(packet)

    def cmd_hover(self, msg):
        packet = self.packer.send_hover_setpoint(msg.vx, msg.vy, msg.yawrate, msg.z_distance)
        self.send_crtp_async(packet)

    def cmd_full_state(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        acc = [msg.acc.x, msg.acc.y, msg.acc.z]
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w ]
        rollRate = msg.twist.angular.x
        pitchRate = msg.twist.angular.y
        yawRate = msg.twist.angular.z
        packet = self.packer.send_full_state_setpoint(pos, vel, acc, orientation, rollRate, pitchRate, yawRate)
        self.send_crtp_async(packet)


    def cmd_position(self, msg):
        packet = self.packer.send_position_setpoint(msg.x, msg.y, msg.z, msg.yaw)
        self.send_crtp_async(packet)

