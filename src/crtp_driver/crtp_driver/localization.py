from crtp_driver.crtp_packer import CrtpPacker
from crtplib.logic.localization_logic import LocalizationLogic

class Localization(LocalizationLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPacker, CrtpLink)





