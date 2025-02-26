cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(crtp_link)

include_directories(include)
add_library(crtp_link 
        crtp_link.cpp)

