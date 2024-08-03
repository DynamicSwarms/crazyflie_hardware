import struct

from crtplib.utils.encoding import compress_quaternion
from .packer import Packer

class GenericCommanderPacker(Packer):
    """
    Used for sending control setpoints to the Crazyflie
    """
    PORT_COMMANDER_GENERIC = 0x07

    SET_SETPOINT_CHANNEL = 0
    META_COMMAND_CHANNEL = 1

    TYPE_STOP = 0
    TYPE_VELOCITY_WORLD = 1
    TYPE_ZDISTANCE = 2
    TYPE_HOVER = 5
    TYPE_FULL_STATE = 6
    TYPE_POSITION = 7

    TYPE_META_COMMAND_NOTIFY_SETPOINT_STOP = 0

    def __init__(self, CrtpPacker):
        super().__init__(CrtpPacker, self.PORT_COMMANDER_GENERIC)

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
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL,
                                    data=data)

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
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL,
                                    data=data)

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
