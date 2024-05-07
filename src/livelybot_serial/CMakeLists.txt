cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    include/hardware/robot.h
    include/hardware/motor.h
    include/robot_node.h
    src/lively_serial.cc
    src/hardware/motor.cc
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial
)
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# test
add_executable(lvbot_test src/serial_t.cpp ${serial_SRCS})
target_link_libraries(lvbot_test ${catkin_LIBRARIES})

add_executable(ik_run src/ik_run.cpp ${serial_SRCS})
target_link_libraries(ik_run ${catkin_LIBRARIES})

add_executable(robot_node_ src/robot_node.cpp ${serial_SRCS})
target_link_libraries(robot_node_ ${catkin_LIBRARIES})


add_executable(test_2dof_ankle src/test_2dof_ankle.cpp ${serial_SRCS})
target_link_libraries(test_2dof_ankle ${catkin_LIBRARIES})

add_executable(test_motor_run src/test_motor_run.cpp ${serial_SRCS})
target_link_libraries(test_motor_run ${catkin_LIBRARIES})

add_executable(test_motor_run_18 src/test_motor_run_18.cpp ${serial_SRCS})
target_link_libraries(test_motor_run_18 ${catkin_LIBRARIES})

add_executable(test_motor_run_20 src/test_motor_run_20.cpp ${serial_SRCS})
target_link_libraries(test_motor_run_20 ${catkin_LIBRARIES})

add_executable(test_motor src/test_motor.cpp ${serial_SRCS})
target_link_libraries(test_motor ${catkin_LIBRARIES})

add_executable(test_motor_torque src/test_motor_torque.cpp ${serial_SRCS})
target_link_libraries(test_motor_torque ${catkin_LIBRARIES})

add_executable(test_motor_combi_pos_vel src/test_motor_combi_pos_vel.cpp ${serial_SRCS})
target_link_libraries(test_motor_combi_pos_vel ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/hardware/robot.h include/hardware/motor.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial)
