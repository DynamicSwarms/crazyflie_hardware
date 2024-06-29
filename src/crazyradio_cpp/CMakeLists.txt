cmake_minimum_required(VERSION 3.8)
project(crazyradio_cpp)

add_subdirectory(deps/libcrazyradio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(crtp_interface REQUIRED)

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

include_directories(
  deps/libcrazyradio/include
)

add_executable(crazyradio src/crazyradio.cpp)
ament_target_dependencies(crazyradio
  rclcpp
  crtp_interface)


install(TARGETS crazyradio DESTINATION lib/${PROJECT_NAME})

target_link_libraries(crazyradio libcrazyradio)

ament_package()
