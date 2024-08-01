"""
Used for sending control setpoints to the Crazyflie
"""
import struct
import numpy as np

from crtp_interface.msg import CrtpPacket
from crtp_driver.crtp_packer import CrtpPacker

from cflib.utils.encoding import compress_quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from crazyflie_interface.msg import NotifySetpointsStop, VelocityWorld, Hover, FullState, Position # (Generic)Commander

__author__ = 'Bitcraze AB'
__all__ = ['Commander']


class GenericCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = GenericCommanderPacker()

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


class GenericCommanderPacker(CrtpPacker):
    """
    Used for sending control setpoints to the Crazyflie
    """

    SET_SETPOINT_CHANNEL = 0
    META_COMMAND_CHANNEL = 1

    TYPE_STOP = 0
    TYPE_VELOCITY_WORLD = 1
    TYPE_ZDISTANCE = 2
    TYPE_HOVER = 5
    TYPE_FULL_STATE = 6
    TYPE_POSITION = 7

    TYPE_META_COMMAND_NOTIFY_SETPOINT_STOP = 0

    PORT_COMMANDER_GENERIC = 0x07

    def __init__(self):
        super().__init__(self.PORT_COMMANDER_GENERIC)

    def send_notify_setpoint_stop(self, remain_valid_milliseconds=0):
        """
        Sends a packet so that the priority of the current setpoint to the lowest non-disabled value,
        so any new setpoint regardless of source will overwrite it.
        """
        data = struct.pack('<BI', self.TYPE_META_COMMAND_NOTIFY_SETPOINT_STOP,
                              remain_valid_milliseconds)
        return self._prepare_packet(channel=self.META_COMMAND_CHANNEL,
                                    data=data)

    def send_stop_setpoint(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        data = struct.pack('<B', self.TYPE_STOP)
        return self._prepare_packet(data=data)

    def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
        """
        Send Velocity in the world frame of reference setpoint with yawrate commands

        vx, vy, vz are in m/s
        yawrate is in degrees/s
        """
        data = struct.pack('<Bffff', self.TYPE_VELOCITY_WORLD,
                              vx, vy, vz, yawrate)
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL,
                                    data=data)

    def send_zdistance_setpoint(self, roll, pitch, yawrate, zdistance):
        """
        Control mode where the height is send as an absolute setpoint (intended
        to be the distance to the surface under the Crazflie), while giving roll,
        pitch and yaw rate commands

        roll, pitch are in degrees
        yawrate is in degrees/s
        zdistance is in meters
        """
        data = struct.pack('<Bffff', self.TYPE_ZDISTANCE,
                              roll, pitch, yawrate, zdistance)
        self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL,
                             data=data)

    def send_hover_setpoint(self, vx, vy, yawrate, zdistance):
        """
        Control mode where the height is send as an absolute setpoint (intended
        to be the distance to the surface under the Crazflie), while giving x, y velocity
        commands in body-fixed coordinates.

        vx,  vy are in m/s
        yawrate is in degrees/s
        zdistance is in meters
        """
        data = struct.pack('<Bffff', self.TYPE_HOVER,
                              vx, vy, yawrate, zdistance)
        self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL,
                             data=data)

    def send_full_state_setpoint(self, pos, vel, acc, orientation, rollrate, pitchrate, yawrate):
        """
        Control mode where the position, velocity, acceleration, orientation and angular
        velocity are sent as absolute (world) values.

        position [x, y, z] are in m
        velocity [vx, vy, vz] are in m/s
        acceleration [ax, ay, az] are in m/s^2
        orientation [qx, qy, qz, qw] are the quaternion components of the orientation
        rollrate, pitchrate, yawrate are in degrees/s
        """
        def vector_to_mm_16bit(vec):
            return int(vec[0] * 1000), int(vec[1] * 1000), int(vec[2] * 1000)

        x, y, z = vector_to_mm_16bit(pos)
        vx, vy, vz = vector_to_mm_16bit(vel)
        ax, ay, az = vector_to_mm_16bit(acc)
        rr, pr, yr = vector_to_mm_16bit([rollrate, pitchrate, yawrate])
        orient_comp = compress_quaternion(orientation)

        data = struct.pack('<BhhhhhhhhhIhhh', self.TYPE_FULL_STATE,
                            x, y, z,
                            vx, vy, vz,
                            ax, ay, az,
                            orient_comp,
                            rr, pr, yr)
        return self._prepare_packet(data=data)

    def send_position_setpoint(self, x, y, z, yaw):
        """
        Control mode where the position is sent as absolute (world) x,y,z coordinate in
        meter and the yaw is the absolute orientation.

        x, y, z are in m
        yaw is in degrees
        """
        data = struct.pack('<Bffff', self.TYPE_POSITION,
                            x, y, z, yaw)
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL,
                                    data=data)

    def compress_quaternion(quat):
        """Compress a quaternion.
        assumes input quaternion is normalized. will fail if not.

        see quatcompress.h in firmware the for definitions

        Args:
            quat : An array of floats representing a quaternion [x, y, z, w]

        Returns: 32-bit integer
        """
        # Normalize the quaternion
        quat_n = np.array(quat) / np.linalg.norm(quat)

        i_largest = 0
        for i in range(1, 4):
            if abs(quat_n[i]) > abs(quat_n[i_largest]):
                i_largest = i
        negate = quat_n[i_largest] < 0

        M_SQRT1_2 = 1.0 / np.sqrt(2)

        comp = i_largest
        for i in range(4):
            if i != i_largest:
                negbit = int((quat_n[i] < 0) ^ negate)
                mag = int(((1 << 9) - 1) * (abs(quat_n[i]) / M_SQRT1_2) + 0.5)
                comp = (comp << 10) | (negbit << 9) | mag

        return comp    