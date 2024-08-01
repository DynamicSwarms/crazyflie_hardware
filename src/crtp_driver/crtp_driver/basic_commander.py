#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2023 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Used for sending control setpoints to the Crazyflie
"""
import struct
import numpy as np

from crtp_interface.msg import CrtpPacket
from crtp_driver.crtp_packer import CrtpPacker

from cflib.utils.encoding import compress_quaternion

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class BasicCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = BasicCommanderPacker()

        callback_group = MutuallyExclusiveCallbackGroup()

        # TODO: sendSetpoint not implemented yet
        #node.create_subscription(SendSetpoint, "~/send_setpoint", self.send_setpoint, 10,callback_group=callback_group) 

    def send_setpoint(self, msg):
        packet = self.packer(msg.roll, msg.pitch, msg.yawrate, msg.thrust)
        self.send_crtp_async(packet)

class BasicCommanderPacker(CrtpPacker):
    """
    Used for sending control setpoints to the Crazyflie
    """
    PORT_BASIC_COMMANDER = 0x03
    
    def __init__(self):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        super().__init__(self.PORT_BASIC_COMMANDER)
        self._x_mode = False

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    def send_setpoint(self, roll, pitch, yawrate, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw_Rate/thrust to the copter.

        The meaning of these values is depended on the mode of the RPYT commander in the firmware
        Default settings are Roll, pitch, yawrate and thrust

        roll,  pitch are in degrees
        yawrate is in degrees/s
        thrust is an integer value ranging from 10001 (next to no power) to 60000 (full power)
        """
        if thrust > 0xFFFF or thrust < 0:
            raise ValueError('Thrust must be between 0 and 0xFFFF')

        if self._x_mode:
            roll, pitch = 0.707 * (roll - pitch), 0.707 * (roll + pitch)

        data = struct.pack('<fffH', roll, -pitch, yawrate, thrust)
        return self._prepare_packet(data=data)
