import logging

logger = logging.getLogger(__name__)

class RadioPacket:
    def __init__(self, data, response_bytes):
        self.data = data
        self.response_bytes = response_bytes

    def matches_response(self, resp):
        l = self.response_bytes + 1 ## port/channel shall always match
        if self.data[:l] == resp[:l]:
            return True
        return False


class Link:
    def __init__(self, channel, address, datarate):
        self.channel = channel
        self.address = address
        self.datarate = datarate
        self.out_packets = []
        self.wd_packets = []

    def add_packet(self, packet, done_event):
        self.out_packets.append([packet,  done_event])
        self.wd_packets.append([packet, done_event])


    def handle_response(self, response):
        lost = [] # Detect lost packets by Order (CRTP should strictly order in Channel) TODO: ChannelORdering
        for pklst in self.wd_packets:
            pk = pklst[0]
            if pk.matches_response(response):
                pk.response = response # write response
                pklst[1].set()      # set done_event                    
                self.wd_packets.remove(pklst)
                break
            else:
                lost.append(pklst)
        else:   # Reponse is not interesting for us
            return

        for lst in lost:
            logger.info("Lost Packet Detected: " + str(lst[0].data))
            self.wd_packets.remove(lst)
            self.out_packets.append(lst)
            self.wd_packets.append(lst)



    def matches(self, l):
        if (self.channel == l.channel and
            self.address == l.address and
            self.datarate == l.datarate):
            return True
        return False



class RadioQueue:

    def __init__(self):
        self.links = []

    def addRadioPacket(self, link, packet,done_event):
        for l in self.links:
            if l.matches(link):
                link = l
                break
        else:
            self.links.append(link)
        link.add_packet(packet, done_event)
    
    def getRadioPacket(self):
        for link in self.links: # TODO choose better permutation each time for fairness
            if len(link.out_packets):
                return link, link.out_packets.pop(0)[0].data
        return None, None
    