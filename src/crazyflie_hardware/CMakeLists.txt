cmake_minimum_required(VERSION 3.10)
project(crazyflie_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(crtp_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(crtp_interfaces REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(object_tracker_interfaces REQUIRED)
find_package(broadcaster_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(crazyflie src/crazyflie.cpp)
add_executable(broadcaster src/broadcaster.cpp)
add_executable(position_converter src/position_converter.cpp)
target_include_directories(crazyflie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(crazyflie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(broadcaster PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(position_converter PUBLIC cxx_std_17)



set(CRTP_DRIVER_SRCS
  src/crtp_driver_cpp/hl_commander.cpp
  src/crtp_driver_cpp/generic_commander.cpp
  src/crtp_driver_cpp/parameters.cpp
  src/crtp_driver_cpp/logging.cpp
  src/crtp_driver_cpp/logblock.cpp
  src/crtp_driver_cpp/console.cpp  
  src/crtp_driver_cpp/localization.cpp
  src/crtp_driver_cpp/platform.cpp
  src/crtp_driver_cpp/link_layer.cpp
  src/crtp_link_ros.cpp
)

add_library(crtp_driver_cpp ${CRTP_DRIVER_SRCS})
set_target_properties(crtp_driver_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(crtp_driver_cpp PUBLIC cxx_std_17)
target_include_directories(crtp_driver_cpp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
target_link_libraries(crtp_driver_cpp
  rclcpp::rclcpp
  crtp_cpp::crtp_cpp
  crazyflie_interfaces::crazyflie_interfaces
  object_tracker_interfaces::object_tracker_interfaces
  broadcaster_interfaces::broadcaster_interfaces
  crtp_interfaces::crtp_interfaces
  rclcpp_lifecycle::rclcpp_lifecycle)

install(TARGETS crtp_driver_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

target_link_libraries(crazyflie
                    crtp_driver_cpp)
target_link_libraries(broadcaster
                    crtp_driver_cpp)

target_link_libraries(crazyflie
  crtp_cpp::crtp_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  crtp_interfaces::crtp_interfaces
  rclcpp_lifecycle::rclcpp_lifecycle
  lifecycle_msgs::lifecycle_msgs)

target_link_libraries(broadcaster
  crtp_cpp::crtp_cpp
  rclcpp::rclcpp
  crtp_interfaces::crtp_interfaces
  broadcaster_interfaces::broadcaster_interfaces)

target_link_libraries(position_converter
  rclcpp::rclcpp
  crazyflie_interfaces::crazyflie_interfaces
  object_tracker_interfaces::object_tracker_interfaces)

install(TARGETS crazyflie
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS position_converter
  DESTINATION lib/${PROJECT_NAME})

## Also register crazyflie as component
# create ament index resource which references the libraries in the binary dir
add_library(crazyflie_component SHARED src/crazyflie.cpp)
target_compile_definitions(crazyflie_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(crazyflie_component crtp_driver_cpp)
target_link_libraries(crazyflie_component
  rclcpp::rclcpp
  crtp_cpp::crtp_cpp
  rclcpp_components::component)

rclcpp_components_register_nodes(crazyflie_component "CrazyflieNode")
install(TARGETS crazyflie_component DESTINATION lib)

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
