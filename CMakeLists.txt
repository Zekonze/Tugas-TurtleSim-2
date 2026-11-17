cmake_minimum_required(VERSION 3.0.2)
project(turtle_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  turtlesim
)

add_message_files(FILES MotionGoal.msg MissionStatus.msg WallSensor.msg HuntScore.msg)
add_service_files(FILES StartMission.srv)
generate_messages()

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime rospy turtlesim)

catkin_install_python(PROGRAMS
  scripts/mission_manager.py
  scripts/motion_controller.py
  scripts/wall_sensor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
