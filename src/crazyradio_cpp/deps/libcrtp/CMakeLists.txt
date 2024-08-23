cmake_minimum_required(VERSION 3.5)
project(libcrtp)

include_directories(include)
add_library(libcrtp 
        CrtpPacketQueue.cpp
        CrtpLink.cpp)

