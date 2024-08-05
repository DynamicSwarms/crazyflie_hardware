from crtplib.packers.basic_commander import BasicCommanderPacker

class BasicCommanderLogic:
    """
    Used for sending control setpoints to the Crazyflie
    """
    def __init__(self, CrtpPacker, CrtpLink):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self.link = CrtpLink
        self.packer = BasicCommanderPacker(CrtpPacker)
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

        packet = self.packer.send_setpoint(roll, -pitch, yawrate, thrust)
        self.link.send_packet_no_response(packet)