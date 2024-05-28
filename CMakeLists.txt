cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GSL REQUIRED)

set(WIRINGPI_INCLUDE_DIRS /usr/include)
set(WIRINGPI_LIBRARIES /usr/lib/libwiringPi.so)


#include dirs
include_directories(include ${GSL_INCLUDE_DIRS} ${WIRINGPI_INCLUDE_DIRS})


add_executable(state_machine_sub
  src/state_machine.cpp
  src/bezier.cpp
  src/capit.cpp
  src/dh.cpp
  src/ik.cpp
  src/interrupt.cpp
  src/leg.cpp
  src/move.cpp
  src/pwm_servo.cpp
)


# Link GSL and WiringPi libraries
target_link_libraries(state_machine_sub ${GSL_LIBRARIES} ${WIRINGPI_LIBRARIES})

#dependencies
ament_target_dependencies(state_machine_sub rclcpp std_msgs)

install(TARGETS
  state_machine_sub
  DESTINATION lib/state_machine
)

ament_package()
