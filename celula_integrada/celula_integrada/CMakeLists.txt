cmake_minimum_required(VERSION 3.0.2)
project(celula_integrada)

find_package(catkin REQUIRED COMPONENTS rospy actionlib actionlib_msgs message_generation)

add_action_files(FILES TransferPiece.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS rospy actionlib actionlib_msgs message_runtime)

catkin_install_python(PROGRAMS
  scripts/maestro.py
  scripts/tm_node.py
  scripts/ordenacao.py
  scripts/analisar_log.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
