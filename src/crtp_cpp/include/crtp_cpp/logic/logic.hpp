#pragma once

#include "crtp_cpp/link/crtp_link.hpp" // Include your CrtpLink header
#include <memory>

class Logic {
public:
    explicit Logic(std::shared_ptr<CrtpLink> crtp_link);
    virtual ~Logic() = default; // Important: Virtual destructor for ABC

protected:
    std::shared_ptr<CrtpLink> link;
};
