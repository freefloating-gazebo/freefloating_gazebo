cmake_minimum_required(VERSION 2.8.3)
project(freefloating_gazebo)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(gazebo)

if(gazebo_FOUND)
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    std_srvs
    gazebo_msgs
    geometry_msgs
    sensor_msgs
    urdf
    control_toolbox
    rostime
    nav_msgs
    message_generation
    )
else()
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  urdf
  control_toolbox
  rostime
  nav_msgs
  message_generation
)
endif()


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


add_service_files(
  FILES
  ControlType.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    gazebo_msgs
    geometry_msgs
    sensor_msgs
    control_toolbox
    rostime
    nav_msgs
    message_runtime
)

include_directories(include ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})



add_library(freefloating_gazebo
  include/freefloating_gazebo/butterworth.h
    include/freefloating_gazebo/hydro_link.h src/hydro_link.cpp
    include/freefloating_gazebo/hydro_model_parser.h src/hydro_model_parser.cpp
  include/freefloating_gazebo/thruster_allocator.h src/thruster_allocator.cpp)
target_link_libraries(freefloating_gazebo ${catkin_LIBRARIES})



# node to perform PID control for body (thrusters) and joints
add_executable(pid_control src/freefloating_pids_main.cpp
    include/freefloating_gazebo/butterworth.h
    src/freefloating_pids.cpp include/freefloating_gazebo/freefloating_pids.h
    src/freefloating_pids_body.cpp include/freefloating_gazebo/freefloating_pids_body.h
    src/freefloating_pids_joint.cpp include/freefloating_gazebo/freefloating_pids_joint.h)
target_link_libraries(pid_control ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(pid_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(pid_control freefloating_gazebo_gencpp)


if(gazebo_FOUND)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

# world plugin to simulate buoyancy and viscous force
add_library(freefloating_gazebo_fluid src/freefloating_gazebo_fluid.cpp include/freefloating_gazebo/freefloating_gazebo_fluid.h)
target_link_libraries(freefloating_gazebo_fluid ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# model plugin subscribes to body and joint efforts and applies them in Gazebo
add_library(freefloating_gazebo_control src/freefloating_gazebo_control.cpp
                                        include/freefloating_gazebo/freefloating_gazebo_control.h)
target_link_libraries(freefloating_gazebo_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROJECT_NAME})

install(TARGETS freefloating_gazebo_fluid
                freefloating_gazebo_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY world
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
        
        
# test Butterworth filter
add_executable(butter test/testButter.cpp)
target_link_libraries(butter ${catkin_LIBRARIES})

# test thruster mapper
add_executable(sdf_parse test/sdf_parse.cpp)
target_link_libraries(sdf_parse ${catkin_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})  

endif()


install(TARGETS pid_control ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY scripts
                  srv
                  gui
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

