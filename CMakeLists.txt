cmake_minimum_required(VERSION 2.8.3)
project(niks_experiments)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS message_generation rospy)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(pocketsphinx REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        pluginlib
        geometric_shapes
        control_msgs
        franka_gripper
        control_msgs
        rosgraph_msgs
        rospy
)



find_package(Boost REQUIRED system filesystem date_time thread)


set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include

)

catkin_package()


add_executable(Z_FirstTry src/Z_FirstTry.cpp)
target_link_libraries(Z_FirstTry ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(Stretching src/Stretching.cpp)
target_link_libraries(Stretching ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARY})

add_executable(StretchingSpeech src/StretchingSpeech.cpp)
target_link_libraries(StretchingSpeech ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARY})

add_executable(Stretching_ohne_Rviz src/Stretching_ohne_Rviz.cpp)
target_link_libraries(Stretching_ohne_Rviz ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(Kick_Off_Event_Move_It src/Kick_Off_Event_Move_It.cpp)
target_link_libraries(Kick_Off_Event_Move_It ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ErrorReset src/ErrorReset.cpp)
target_link_libraries(ErrorReset ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY franka_ros/include/franka_gripper/franka_gripper.h
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.h"
  	PATTERN ".svn" EXCLUDE)


install(PROGRAMS scripts/opc_ua_ros_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(TARGETS Z_FirstTry DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS Stretching DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS StretchingSpeech DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS Stretching_ohne_Rviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS Kick_Off_Event_Move_It DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ErrorReset DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
