
from crtp_interface.msg import CrtpResponse

class ParamReader():

    def __init__(self, node):
        node.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  10)
        self.node = node
        self.count = 0
    def set_count(self, c):
        self.count = c
    def handle_response(self, response):
        data = response.packet.data
        data_length = response.packet.data_length
        channel = response.packet.channel
        port = response.packet.port

        if port != 2 or channel != 0 or not data_length: return # not us 

        name = data[4:data_length]
        
        if len(name): 
            string = "New log_toc entry: '"
            for ch in name:
                if ch == 0: string = string + " " 
                else: string = string + chr(ch)
            string = string + "'"
            self.node.get_logger().info(string)
            self.count = self.count - 1
        if (self.count > 0): self.get_next()
    def get_next(self):
        req = self.node._prepare_send_request()
        CMD_TOC_ITEM_V2 = 2  # version 2: up to 16k entries
        
        req.packet.port = 2 # log
        req.packet.channel = 0 # access
        req.packet.data_length = 2

        req.packet.data[0] = CMD_TOC_ITEM_V2
        req.packet.data[1] = self.count
        self.node.send_packet_service.call_async(req)
        self.node.get_logger().info("requesting")
