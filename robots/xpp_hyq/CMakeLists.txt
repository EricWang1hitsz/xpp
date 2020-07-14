cmake_minimum_required(VERSION 2.8.3)
project(xpp_hyq)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_vis
  laikago_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/hyqleg_inverse_kinematics.cc
  src/leg_inverse_kinematics.cc # leg inverse kinematics for hitsz quadruped.
  src/laikaleg_inverse_kinematics.cc # leg inverse kinematics for laikago robot.
  src/inverse_kinematics_hyq1.cc
  src/inverse_kinematics_hyq2.cc
  src/inverse_kinematics_hyq4.cc
  src/inverse_kinematics_bot.cc
  src/inverse_kinematics_laikago.cc
)

## URDF visualizers for all HyQ variants
add_executable(urdf_visualizer_hyq1 src/exe/urdf_visualizer_hyq1.cc)
target_link_libraries(urdf_visualizer_hyq1
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq2 src/exe/urdf_visualizer_hyq2.cc)
target_link_libraries(urdf_visualizer_hyq2
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq4 src/exe/urdf_visualizer_hyq4.cc)
target_link_libraries(urdf_visualizer_hyq4
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
# URDF visualizers for hitsz quadruped
add_executable(urdf_visualizer_bot src/exe/urdf_visualizer_bot.cc)
target_link_libraries(urdf_visualizer_bot
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
# URDF visualizers for laikago.
add_executable(urdf_visualizer_laikago src/exe/urdf_visualizer_laikago.cc)
target_link_libraries(urdf_visualizer_laikago
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(laikago_joints_publisher_node src/laikago_joints_pub.cc)
target_link_libraries(laikago_joints_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} urdf_visualizer_hyq1 urdf_visualizer_hyq2 urdf_visualizer_hyq4 urdf_visualizer_bot urdf_visualizer_laikago
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch rviz meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
