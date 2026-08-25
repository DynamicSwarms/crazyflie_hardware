#pragma once

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string_view>

namespace crazyflie_hardware
{

inline std::filesystem::path ros_home()
{
    const char * ros_home = std::getenv("ROS_HOME");
    if (ros_home != nullptr && ros_home[0] != '\0')
    {
        return ros_home;
    }

    const char * home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0')
    {
        throw std::runtime_error("Cannot determine ROS home: ROS_HOME and HOME are unset");
    }
    return std::filesystem::path(home) / ".ros";
}

inline std::filesystem::path toc_cache_path(std::string_view category)
{
    return ros_home() / "crazyflie_hardware" / "toc" / std::string(category);
}

}  // namespace crazyflie_hardware
