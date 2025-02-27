#include "crtp_cpp/link/crtp_link.hpp"

CrtpLink::CrtpLink(int channel, std::tuple<int> address, int datarate)
    : channel(channel), address(address), datarate(datarate) {}

//  No implementations needed here for pure virtual functions.  They *must* be implemented
//  in derived classes.  It's an abstract base class.