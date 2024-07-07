cmake_minimum_required(VERSION 3.5)
project(libcrazyradio)
find_library(USB_LIB usb-1.0)

include_directories(include)
add_library(libcrazyradio 
        Crazyradio.cpp
        USBDevice.cpp
        CrtpPacketQueue.cpp
        CrtpLink.cpp)

target_link_libraries(libcrazyradio
    PRIVATE
    ${USB_LIB})
