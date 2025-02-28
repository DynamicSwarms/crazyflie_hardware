cmake_minimum_required(VERSION 3.8)
project(crazyflie_hardware_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(crtp_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crtp_interfaces REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(crazyflie src/crazyflie.cpp)
target_include_directories(crazyflie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(crazyflie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



set(CRTP_DRIVER_SRCS
  src/crtp_driver_cpp/hl_commander.cpp
  src/crtp_link_ros.cpp
)

add_library(crtp_driver_cpp ${CRTP_DRIVER_SRCS})
target_compile_features(crtp_driver_cpp PUBLIC cxx_std_17)
target_include_directories(crtp_driver_cpp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
ament_target_dependencies(crtp_driver_cpp 
  rclcpp
  crtp_cpp
  crazyflie_interfaces
  crtp_interfaces
  rclcpp_lifecycle)

target_link_libraries(crazyflie
                    crtp_driver_cpp)

ament_target_dependencies(crazyflie
  crtp_cpp
  rclcpp
  crtp_interfaces
  rclcpp_lifecycle
  lifecycle_msgs)


install(TARGETS crazyflie
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
