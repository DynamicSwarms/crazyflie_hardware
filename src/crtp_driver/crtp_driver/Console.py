from crtp_driver.crtp_packer import CrtpPacker
from crtplib.logic.console_logic import ConsoleLogic

class Console(ConsoleLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPacker, CrtpLink)

    