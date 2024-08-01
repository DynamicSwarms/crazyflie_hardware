import struct 
import numpy as np

from crtp_driver.crtp_packer import CrtpPacker

from crazyflie_interface.msg import SetGroupMask, Takeoff, Land, Stop, GoTo, StartTrajectory, UploadTrajectory # HL_Commander
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HighLevelCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = HighLevelCommanderPacker()

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


class HighLevelCommanderPacker(CrtpPacker):
    PORT_HL_COMMANDER = 0x08
    CHANNEL_HL_COMMANDER = 0

    COMMAND_SET_GROUP_MASK = 0
    COMMAND_STOP = 3
    COMMAND_GO_TO = 4
    COMMAND_START_TRAJECTORY = 5
    COMMAND_DEFINE_TRAJECTORY = 6
    COMMAND_TAKEOFF_2 = 7
    COMMAND_LAND_2 = 8

    ALL_GROUPS = 0

    TRAJECTORY_LOCATION_MEM = 1

    TRAJECTORY_TYPE_POLY4D = 0
    TRAJECTORY_TYPE_POLY4D_COMPRESSED = 1

    def __init__(self):
        super().__init__(self.PORT_HL_COMMANDER)

    def _prepare_packet(self, data):
        return super()._prepare_packet(channel=self.CHANNEL_HL_COMMANDER,
                                      data=data)

    def set_group_mask(self, group_mask=ALL_GROUPS):
        """
        Set the group mask that the Crazyflie belongs to

        :param group_mask: Mask for which groups this CF belongs to
        """
        data = struct.pack('<BB',
                           self.COMMAND_SET_GROUP_MASK, 
                           group_mask)
        return self._prepare_packet(data)

    def takeoff(self, absolute_height_m, duration_s, group_mask=ALL_GROUPS, yaw=0.0):
        """
        vertical takeoff from current x-y position to given height

        :param absolute_height_m: Absolute (m)
        :param duration_s: Time it should take until target height is
                           reached (s)
        :param group_mask: Mask for which CFs this should apply to
        :param yaw: Yaw (rad). Use current yaw if set to None.
        """
        target_yaw = yaw
        useCurrentYaw = False
        if yaw is None:
            target_yaw = 0.0
            useCurrentYaw = True
        data = struct.pack('<BBff?f',
                            self.COMMAND_TAKEOFF_2,
                            group_mask,
                            absolute_height_m,
                            target_yaw,
                            useCurrentYaw,
                            duration_s)
        return self._prepare_packet(data)
    
    def land(self, absolute_height_m, duration_s, group_mask=ALL_GROUPS,
             yaw=0.0):
        """
        vertical land from current x-y position to given height

        :param absolute_height_m: Absolute (m)
        :param duration_s: Time it should take until target height is
                           reached (s)
        :param group_mask: Mask for which CFs this should apply to
        :param yaw: Yaw (rad). Use current yaw if set to None.
        """
        target_yaw = yaw
        useCurrentYaw = False
        if yaw is None:
            target_yaw = 0.0
            useCurrentYaw = True
        data = struct.pack('<BBff?f',
                            self.COMMAND_LAND_2,
                            group_mask,
                            absolute_height_m,
                            target_yaw,
                            useCurrentYaw,
                            duration_s)
        return self._prepare_packet(data)

    def stop(self, group_mask=ALL_GROUPS):
        """
        stops the current trajectory (turns off the motors)

        :param group_mask: Mask for which CFs this should apply to
        :return:
        """
        data = struct.pack('<BB',
                            self.COMMAND_STOP,
                            group_mask)
        return self._prepare_packet(data)
    
    def go_to(self, x, y, z, yaw, duration_s, relative=False,
              group_mask=ALL_GROUPS):
        """
        Go to an absolute or relative position

        :param x: X (m)
        :param y: Y (m)
        :param z: Z (m)
        :param yaw: Yaw (radians)
        :param duration_s: Time it should take to reach the position (s)
        :param relative: True if x, y, z is relative to the current position
        :param group_mask: Mask for which CFs this should apply to
        """
        data = struct.pack('<BBBfffff',
                                      self.COMMAND_GO_TO,
                                      group_mask,
                                      relative,
                                      x, y, z,
                                      yaw,
                                      duration_s)
        return self._prepare_packet(data)

    def start_trajectory(self, trajectory_id, time_scale=1.0, relative=False,
                         reversed=False, group_mask=ALL_GROUPS):
        """
        starts executing a specified trajectory

        :param trajectory_id: Id of the trajectory (previously defined by
               define_trajectory)
        :param time_scale: Time factor; 1.0 = original speed;
                                        >1.0: slower;
                                        <1.0: faster
        :param relative: Set to True, if trajectory should be shifted to
               current setpoint
        :param reversed: Set to True, if trajectory should be executed in
               reverse
        :param group_mask: Mask for which CFs this should apply to
        :return:
        """
        data = struct.pack('<BBBBBf',
                                      self.COMMAND_START_TRAJECTORY,
                                      gsrc/crtp_driver/crtp_driver/commander.pyroup_mask,
                                      relative,
                                      reversed,
                                      trajectory_id,
                                      time_scale)
        return self._prepare_packet(data)

    def define_trajectory(self, trajectory_id, offset, n_pieces, type=TRAJECTORY_TYPE_POLY4D):
        """
        Define a trajectory that has previously been uploaded to memory.

        :param trajectory_id: The id of the trajectory
        :param offset: Offset in uploaded memory
        :param n_pieces: Nr of pieces in the trajectory
        :param type: The type of trajectory data; TRAJECTORY_TYPE_POLY4D or TRAJECTORY_TYPE_POLY4D_COMPRESSED
        :return:
        """
        data = struct.pack('<BBBBIB',
                                      self.COMMAND_DEFINE_TRAJECTORY,
                                      trajectory_id,
                                      self.TRAJECTORY_LOCATION_MEM,
                                      type,
                                      offset,
                                      n_pieces)
        return self._prepare_packet(data)