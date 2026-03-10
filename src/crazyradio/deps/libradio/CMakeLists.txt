cmake_minimum_required(VERSION 3.5)
project(libradio)
find_library(USB_LIB usb-1.0)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../libcrtp ${CMAKE_CURRENT_BINARY_DIR}/../libcrtp)

#include_directories(
#  interface
#  crazyradio/include
#  udpradio/include)
add_library(libradio
        crazyradio/Crazyradio.cpp
        crazyradio/USBDevice.cpp
        udpradio/Udpradio.cpp
        udpradio/sitllink.cpp)

target_include_directories(libradio PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/crazyradio/include
  ${CMAKE_CURRENT_LIST_DIR}/udpradio/include
  ${CMAKE_CURRENT_LIST_DIR}/../libcrtp/include
)

target_link_libraries(libradio
    libcrtp
    ${USB_LIB})

target_include_directories(libradio PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../libcrtp/include)
