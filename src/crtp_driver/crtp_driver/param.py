import struct 
import logging
logger = logging.getLogger(__name__)


class Toc:
    """Container for TocElements."""

    def __init__(self):
        self.toc = {}

    def clear(self):
        """Clear the TOC"""
        self.toc = {}

    def add_element(self, element):
        """Add a new TocElement to the TOC container."""
        try:
            self.toc[element.group][element.name] = element
        except KeyError:
            self.toc[element.group] = {}
            self.toc[element.group][element.name] = element

    def get_element_by_complete_name(self, complete_name):
        """Get a TocElement element identified by complete name from the
        container."""
        try:
            return self.get_element_by_id(self.get_element_id(complete_name))
        except ValueError:
            # Item not found
            return None

    def get_element_id(self, complete_name):
        """Get the TocElement element id-number of the element with the
        supplied name."""
        [group, name] = complete_name.split('.')
        element = self.get_element(group, name)
        if element:
            return element.ident
        else:
            logger.warning('Unable to find variable [%s]', complete_name)
            return None

    def get_element(self, group, name):
        """Get a TocElement element identified by name and group from the
        container."""
        try:
            return self.toc[group][name]
        except KeyError:
            return None

    def get_element_by_id(self, ident):
        """Get a TocElement element identified by index number from the
        container."""
        for group in list(self.toc.keys()):
            for name in list(self.toc[group].keys()):
                if self.toc[group][name].ident == ident:
                    return self.toc[group][name]
        return None

class ParamTocElement:
    """An element in the Log TOC."""

    RW_ACCESS = 0
    RO_ACCESS = 1

    EXTENDED_PERSISTENT = 1

    types = {0x08: ('uint8_t', '<B'),
             0x09: ('uint16_t', '<H'),
             0x0A: ('uint32_t', '<L'),
             0x0B: ('uint64_t', '<Q'),
             0x00: ('int8_t', '<b'),
             0x01: ('int16_t', '<h'),
             0x02: ('int32_t', '<i'),
             0x03: ('int64_t', '<q'),
             0x05: ('FP16', ''),
             0x06: ('float', '<f'),
             0x07: ('double', '<d')}

    def __init__(self, ident=0, data=None):
        """TocElement creator. Data is the binary payload of the element."""
        self.ident = ident
        self.persistent = False
        self.extended = False
        if (data):
            strs = struct.unpack('s' * len(data[1:]), data[1:])
            s = ''
            for ch in strs:
                s += ch.decode('ISO-8859-1')
            strs = s.split('\x00')
            self.group = strs[0]
            self.name = strs[1]

            metadata = data[0]
            if isinstance(metadata, str):
                metadata = ord(metadata)

            # If the fouth byte (1 << 4) (0x10) is set we have extended
            # type information for this element.
            self.extended = ((metadata & 0x10) != 0)

            self.ctype = self.types[metadata & 0x0F][0]
            self.pytype = self.types[metadata & 0x0F][1]
            if ((metadata & 0x40) != 0):
                self.access = ParamTocElement.RO_ACCESS
            else:
                self.access = ParamTocElement.RW_ACCESS

    def get_readable_access(self):
        if (self.access == ParamTocElement.RO_ACCESS):
            return 'RO'
        return 'RW'

    def is_extended(self):
        return self.extended

    def mark_persistent(self):
        self.persistent = True

    def is_persistent(self):
        return self.persistent
