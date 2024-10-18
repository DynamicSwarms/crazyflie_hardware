from crtp.packers.parameters_packer import ParametersPacker

from .toc.parameters import ParamTocElement
from .toc_logic import TocLogic

class ParametersLogic(TocLogic):

    def __init__(self, CrtpPacker, CrtpLink, path):
        self.link = CrtpLink
        self.packer = ParametersPacker(CrtpPacker)
        super().__init__(self.packer, self.link, ParamTocElement, path)
           
    def set_parameter(self, group, name, value):
        toc_element = self.toc.get_element(group, name) ## Error checking!!
        id = toc_element.ident
        if toc_element.pytype == '<f' or toc_element.pytype == '<d':
            value_nr = float(value)
        else:
            value_nr = int(value)
        
        packet = self.packer.set_parameter(id, toc_element.pytype, value_nr)
        self.link.send_packet_no_response(packet)
   
        
