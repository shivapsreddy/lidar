cmake_minimum_required(VERSION 3.5)
project(lidar_thermal_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(poco_vendor REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
#find_package(example_interfaces REQUIRED)

include_directories(include)

#  src/lidar_soc_temperature.cpp
add_library(lidar_thermal_cpp SHARED
  src/lidar_soc_temperature_publish.cpp
  src/lidar_soc_temperature_subscriber.cpp)

target_compile_definitions(lidar_thermal_cpp PRIVATE "LIDAR_THERMAL_CPP_DEBUG")

ament_target_dependencies(lidar_thermal_cpp
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "Poco")

rclcpp_components_register_node(lidar_thermal_cpp PLUGIN "lidar_thermal_cpp::Lidar_SoCTemperature" EXECUTABLE lidar_SoCTemperature)
rclcpp_components_register_node(lidar_thermal_cpp PLUGIN "lidar_thermal_cpp::Lidar_SoCTemperaturePublisher" EXECUTABLE lidar_SoCTemperaturePublisher)
rclcpp_components_register_node(lidar_thermal_cpp PLUGIN "lidar_thermal_cpp::Lidar_SoCTemperatureSubscriber" EXECUTABLE lidar_SoCTemperatureSubscriber)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  lidar_thermal_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
