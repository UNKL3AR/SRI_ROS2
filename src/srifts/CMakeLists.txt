cmake_minimum_required(VERSION 3.8)
project(srifts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(SRIFTS_INCLUDE /home/zhao/force_sensor/test_ws/src/srifts/include/srifts)
set(SRI_INTERFACES_INCLUDE /home/zhao/force_sensor/test_ws/install/sri_interface/include/sri_interface)
set(FORCE_SENSOR_SRC src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sri_interface REQUIRED)
find_package(serial REQUIRED)

include_directories(
  include
  ${SRIFTS_INCLUDE}
  ${SRI_INTERFACES_INCLUDE}
)

add_executable(forcesensor ${FORCE_SENSOR_SRC}/M8128_data.cpp)
ament_target_dependencies(forcesensor rclcpp geometry_msgs serial sri_interface)
# target_link_libraries(forcesensor)
 
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

install(TARGETS
  forcesensor
DESTINATION lib/${PROJECT_NAME})

ament_package()
