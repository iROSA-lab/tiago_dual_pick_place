cmake_minimum_required(VERSION 2.8.3)
project(tiago_dual_pick_place)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  rospy
  std_msgs
  std_srvs
)


generate_dynamic_reconfigure_options(
  cfg/Grasps.cfg
  )

add_action_files(
  FILES
  PickPlacePose.action
  PickUpObject.action
  PlaceObject.action
  PlaceAutoObject.action
)

add_service_files(
  FILES
  PickPlaceObject.srv
  PickPlaceObjects.srv
  PickPlaceAutoObject.srv
  PickPlaceSimple.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
        scripts/pick_and_place_server.py
        scripts/pick_client.py
        scripts/grasps_server.py
        scripts/arm_conf.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

