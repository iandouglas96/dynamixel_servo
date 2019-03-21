cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_servo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/dynamixel_servo.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs dynamic_reconfigure
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_servo src/dynamixel_servo.cpp src/dynamixel_servo_node.cpp src/servo.cpp)
add_dependencies(dynamixel_servo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamixel_servo ${catkin_LIBRARIES})
