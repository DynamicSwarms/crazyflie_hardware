from crazyflie_hardware.crtp_driver.crtp_packer_ros import CrtpPackerRos
from crtp.crtp.logic.console_logic import ConsoleLogic

class Console(ConsoleLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPackerRos, CrtpLink)

    