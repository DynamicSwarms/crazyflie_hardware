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


__author__ = 'Bitcraze AB'
__all__ = ['Commander']

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crtp_driver.crtp_packer import CrtpPacker
from crtplib.packers.basic_commander import BasicCommanderPacker

class BasicCommander:
    def __init__(self, node, send_crtp_async=None, send_crtp_sync=None):
        self.send_crtp_sync = send_crtp_sync
        self.send_crtp_async = send_crtp_async

        self.packer = BasicCommanderPacker(CrtpPacker)

        callback_group = MutuallyExclusiveCallbackGroup()

        # TODO: sendSetpoint not implemented yet
        #node.create_subscription(SendSetpoint, "~/send_setpoint", self.send_setpoint, 10,callback_group=callback_group) 

    def send_setpoint(self, msg):
        packet = self.packer(msg.roll, msg.pitch, msg.yawrate, msg.thrust)
        self.send_crtp_async(packet)

