#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017-2023 Bitcraze AB
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
Subsytem handling localization-related data communication
"""
import collections
import logging
import struct

from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.callbacks import Caller
from cflib.utils.encoding import fp16_to_float

from crtp_driver.crtp_packer import CrtpPacker


__author__ = 'Bitcraze AB'
__all__ = ['Localization', 'LocalizationPacket']

logger = logging.getLogger(__name__)

# A generic location packet contains type and data. When received the data
# may be decoded by the lib.
LocalizationPacket = collections.namedtuple('localizationPacket',
                                            ['type', 'raw_data', 'data'])


class Localization(CrtpPacker):
    """
    Handle localization-related data communication with the Crazyflie
    """

    # Implemented channels
    POSITION_CH = 0
    GENERIC_CH = 1

    # Location message types for generig channel
    RANGE_STREAM_REPORT = 0
    RANGE_STREAM_REPORT_FP16 = 1
    LPS_SHORT_LPP_PACKET = 2
    EMERGENCY_STOP = 3
    EMERGENCY_STOP_WATCHDOG = 4
    COMM_GNSS_NMEA = 6
    COMM_GNSS_PROPRIETARY = 7
    EXT_POSE = 8
    EXT_POSE_PACKED = 9
    LH_ANGLE_STREAM = 10
    LH_PERSIST_DATA = 11

    PORT_LOCALIZATION = 6
    def __init__(self, crazyflie=None):
        """
        Initialize the Extpos object.
        """
        super().__init__(self.PORT_LOCALIZATION)

    def _decode_lh_angle(self, data):
        decoded_data = {}

        raw_data = struct.unpack('<Bfhhhfhhh', data)

        decoded_data['basestation'] = raw_data[0]
        decoded_data['x'] = [0, 0, 0, 0]
        decoded_data['x'][0] = raw_data[1]
        decoded_data['x'][1] = raw_data[1] - fp16_to_float(raw_data[2])
        decoded_data['x'][2] = raw_data[1] - fp16_to_float(raw_data[3])
        decoded_data['x'][3] = raw_data[1] - fp16_to_float(raw_data[4])
        decoded_data['y'] = [0, 0, 0, 0]
        decoded_data['y'][0] = raw_data[5]
        decoded_data['y'][1] = raw_data[5] - fp16_to_float(raw_data[6])
        decoded_data['y'][2] = raw_data[5] - fp16_to_float(raw_data[7])
        decoded_data['y'][3] = raw_data[5] - fp16_to_float(raw_data[8])

        return decoded_data

    def send_extpos(self, pos):
        """
        Send the current Crazyflie X, Y, Z position. This is going to be
        forwarded to the Crazyflie's position estimator.
        """
        data = struct.pack('<fff', pos[0], pos[1], pos[2])
        return self._prepare_packet(channel=self.POSITION_CH, data=data)

    def send_extpose(self, pos, quat):
        """
        Send the current Crazyflie pose (position [x, y, z] and
        attitude quaternion [qx, qy, qz, qw]). This is going to be forwarded
        to the Crazyflie's position estimator.
        """

        pk = CRTPPacket()
        pk.port = CRTPPort.LOCALIZATION
        pk.channel = self.GENERIC_CH
        pk.data = struct.pack('<Bfffffff',
                              self.EXT_POSE,
                              pos[0], pos[1], pos[2],
                              quat[0], quat[1], quat[2], quat[3])
        self._cf.send_packet(pk)

    def send_short_lpp_packet(self, dest_id, data):
        """
        Send ultra-wide-band LPP packet to dest_id
        """

        pk = CRTPPacket()
        pk.port = CRTPPort.LOCALIZATION
        pk.channel = self.GENERIC_CH
        pk.data = struct.pack('<BB', self.LPS_SHORT_LPP_PACKET, dest_id) + data
        self._cf.send_packet(pk)

    def send_emergency_stop(self):
        """
        Send emergency stop
        """

        pk = CRTPPacket()
        pk.port = CRTPPort.LOCALIZATION
        pk.channel = self.GENERIC_CH
        pk.data = struct.pack('<B', self.EMERGENCY_STOP)
        self._cf.send_packet(pk)

    def send_emergency_stop_watchdog(self):
        """
        Send emergency stop watchdog
        """

        pk = CRTPPacket()
        pk.port = CRTPPort.LOCALIZATION
        pk.channel = self.GENERIC_CH
        pk.data = struct.pack('<B', self.EMERGENCY_STOP_WATCHDOG)
        self._cf.send_packet(pk)

    def send_lh_persist_data_packet(self, geo_list, calib_list):
        """
        Send geometry and calibration data to persistent memory subsystem
        """

        geo_list.sort()
        calib_list.sort()
        max_bs_nr = 15
        if len(geo_list) > 0:
            if geo_list[0] < 0 or geo_list[-1] > max_bs_nr:
                raise Exception('Geometry BS list is not valid')
        if len(calib_list) > 0:
            if calib_list[0] < 0 or calib_list[-1] > max_bs_nr:
                raise Exception('Calibration BS list is not valid')

        mask_geo = 0
        mask_calib = 0
        for bs in geo_list:
            mask_geo += 1 << bs
        for bs in calib_list:
            mask_calib += 1 << bs

        pk = CRTPPacket()
        pk.port = CRTPPort.LOCALIZATION
        pk.channel = self.GENERIC_CH
        pk.data = struct.pack(
            '<BHH', self.LH_PERSIST_DATA, mask_geo, mask_calib)
        self._cf.send_packet(pk)

        return pk.data
