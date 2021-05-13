cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  std_msgs
  nav_msgs
  message_filters
  geometry_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
  std_srvs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorSpeed.msg
  CustomOdometry.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ResetOdometryToPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  std_srvs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/methods.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES first_project
#  CATKIN_DEPENDS other_catkin_pkg
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_runtime
  message_filters
  geometry_msgs
  std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_executable(scout_velocity src/scout_velocity.cpp)
add_dependencies(scout_velocity first_project_generate_messages_cpp)
target_link_libraries(scout_velocity ${catkin_LIBRARIES})

add_executable(parameters_estimation src/parameters_estimation.cpp)
add_dependencies(parameters_estimation first_project_generate_messages_cpp)
target_link_libraries(parameters_estimation ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry first_project_generate_messages_cpp)
add_dependencies(odometry ${PROJECT_NAME}_gencfg)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(pub_tf2 src/pub_tf2.cpp)
target_link_libraries(pub_tf2 ${catkin_LIBRARIES})

add_executable(pub_scout_tf2 src/pub_scout_tf2.cpp)
target_link_libraries(pub_scout_tf2 ${catkin_LIBRARIES})

add_executable(baseline_estimation src/baseline_estimation.cpp)
add_dependencies(baseline_estimation first_project_generate_messages_cpp)
target_link_libraries(baseline_estimation ${catkin_LIBRARIES})