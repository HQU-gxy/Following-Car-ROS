cmake_minimum_required(VERSION 3.20)
project(stupid_car)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_SYSTEM_SERIAL "Use system serial" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# http://wiki.ros.org/std_msgs
find_package(std_msgs REQUIRED)
# https://index.ros.org/p/geometry_msgs/github-ros2-common_interfaces/
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# https://aur.archlinux.org/packages/serial
# https://github.com/wjwwood/serial
if (USE_SYSTEM_SERIAL)
  find_package(serial REQUIRED)
else()
  add_subdirectory(third-party/serial)
endif()


add_executable(car src/shit.cpp)
target_include_directories(car PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(car rclcpp geometry_msgs nav2_msgs tf2_ros tf2_geometry_msgs)


add_executable(uwb src/uwb.cpp)
target_include_directories(uwb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(uwb rclcpp geometry_msgs tf2_ros)
if (USE_SYSTEM_SERIAL)
  ament_target_dependencies(uwb serial)
else()
  target_include_directories(uwb PRIVATE third-party/serial/include)
  target_link_libraries(uwb serial)
endif()

install(TARGETS car uwb
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
