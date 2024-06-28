import logging 

logger = logging.getLogger(__name__)

class Packet():
    def __init__(self, data, length):
        self.data = data
        self.length = length

class Endpoint():
    def __init__(self, channel, address, datarate):
        self.channel = channel 
        self.address = address
        self.datarate = datarate
        self.packets = []
        self.lost_packets = []


    def matches(self, ep):
        if self.channel == ep.channel and self.address == ep.address and self.datarate == ep.datarate:
            return True
        return False
    
    def add_packet(self, pkt):
        self.packets.append(pkt)
    
    def free_packet(self, data):
        lost = []
        for pk in self.packets: # CRTP Requires strict packet ordering 
            l = pk.length + 1 ## port/channel shall always match
            if data[:l] == pk.data[:l]:
                self.packets.remove(pk)
                break
            else:
                lost.append(pk)
        else:  # Packet needed no Guard
            return
        for lst in lost:
            logger.info("Lost Packet Detected: " + str(lst.data))
            self.lost_packets.append(lst)
            self.packets.remove(lst)

    def get_lost_packets(self):
        ret = self.lost_packets.copy()
        self.lost_packets.clear()
        return ret

class ConnectionWatchdog():
    def __init__(self, send_packet):
        self.send_packet = send_packet
        self.endpoints = []

    def add_packet_to_guard(self, channel, address, datarate, data, length):
        ep = Endpoint(channel, address, datarate)
        pkt = Packet(data, length)
        for endp in self.endpoints:
            if ep.matches(endp):
                endp.add_packet(pkt)
                return
        ep.add_packet(pkt)
        self.endpoints.append(ep)
        
    def free_packet(self, channel, address, datarate, data):
        ep = Endpoint(channel, address, datarate)
        for endp in self.endpoints:
            if ep.matches(endp):
                endp.free_packet(data)

    def resend_lost_packages(self):
        for ep in self.endpoints:
            lost = ep.get_lost_packets()
            for l in lost:
                logger.info("Resending" + str(l.data))
                self.add_packet_to_guard(ep.channel, ep.address, ep.datarate, l.data, l.length)
                self.send_packet(ep.channel, ep.address, ep.datarate, l.data)
           
    def log_lost_packets(self):
        for ep in self.endpoints:
            logger.info(str(ep.get_lost_packets()))


         