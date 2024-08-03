
import struct

from .packer import Packer
from crtplib.utils.encoding import fp16_to_float

class LocalizationPacker(Packer):
    """
    Handle localization-related data communication with the Crazyflie
    """
    PORT_LOCALIZATION = 6

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

    def __init__(self, CrtpPacker):
        """
        Initialize the Extpos object.
        """
        super().__init__(CrtpPacker, self.PORT_LOCALIZATION)

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

        data = struct.pack('<Bfffffff',
                              self.EXT_POSE,
                              pos[0], pos[1], pos[2],
                              quat[0], quat[1], quat[2], quat[3])
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_short_lpp_packet(self, dest_id, data):
        """
        Send ultra-wide-band LPP packet to dest_id
        """

        data = struct.pack('<BB', self.LPS_SHORT_LPP_PACKET, dest_id) + data
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_emergency_stop(self):
        """
        Send emergency stop
        """

        data = struct.pack('<B', self.EMERGENCY_STOP)
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_emergency_stop_watchdog(self):
        """
        Send emergency stop watchdog
        """

        data = struct.pack('<B', self.EMERGENCY_STOP_WATCHDOG)
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

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

        data = struct.pack(
            '<BHH', self.LH_PERSIST_DATA, mask_geo, mask_calib)
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)
