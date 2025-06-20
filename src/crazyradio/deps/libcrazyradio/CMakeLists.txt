cmake_minimum_required(VERSION 3.5)
project(libcrazyradio)
find_library(USB_LIB usb-1.0)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../libcrtp ${CMAKE_CURRENT_BINARY_DIR}/../libcrtp)

include_directories(include)
add_library(libcrazyradio 
        Crazyradio.cpp
        USBDevice.cpp)

target_link_libraries(libcrazyradio
    libcrtp
    ${USB_LIB})

target_include_directories(libcrazyradio PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../libcrtp/include)
