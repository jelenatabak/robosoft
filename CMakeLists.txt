cmake_minimum_required(VERSION 3.0.2)
project(robosoft)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  dynamixel_workbench_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  message_generation
)


catkin_python_setup()

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(yaml-cpp REQUIRED)

add_service_files(FILES
    cartesianPath.srv
    closeGripper.srv
    jointGoal.srv
    poseGoal.srv
    positionGoal.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

set(THIS_PACKAGE_INCLUDE_DIRS
  include
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)



include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})



add_executable(ur_sim 
  src/ur_sim.cpp
)

add_executable(UR5e 
  src/UR5e.cpp
)

add_executable(franka 
  src/franka.cpp
)

target_link_libraries(ur_sim ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ur_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(UR5e ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS UR5e DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(franka ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS franka DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
