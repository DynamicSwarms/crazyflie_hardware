#pragma once

#include "crtp_link/crtp_link.hpp"
#include "crtp_logic/toc_logic.hpp"
#include "crtp_packer/crtp_packer.hpp"
#include "crtp_packer/parameters_packer.hpp"
#include "crtp_logic/toc/parameters.hpp"
#include <string>

struct ParamTocEntry : public TocEntry{
    uint16_t id;
    std::string group;
    std::string name;
    ParamType type;
    bool readonly;
};

/**
 * @brief Logic for parameter-related communication.
 */
class ParametersLogic : public TocLogic {
public:
    /**
     * @brief Constructor for ParametersLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     * @param path Path to the parameter TOC file.
     */
    ParametersLogic(CrtpLink* crtp_link, const std::string& path);
    
    
    

    /**
     * @brief Sends a packet to set a parameter.
     * @param group The parameter group.
     * @param name The parameter name.
     * @param value The parameter value.
     */
    void send_set_parameter(const std::string& group, const std::string& name, double value);

private:
    ParametersPacker packer;
    
    std::vector<ParamTocEntry> toc_entries;
};
