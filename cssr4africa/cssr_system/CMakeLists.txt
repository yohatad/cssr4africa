cmake_minimum_required(VERSION 3.0.2)
project(cssr_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  actionlib
  control_msgs
  image_transport
  message_generation
  utilities
  behaviortree_cpp
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Add service and message files
add_message_files(
  DIRECTORY
  gestureExecution/msg
  FILES
  Gesture.msg
)

add_message_files(
  DIRECTORY
  overtAttention/msg
  FILES
  Mode.msg
  Status.msg
)

add_message_files(
  DIRECTORY
  face_detection/msg
  FILES
  face_detection_msg_file.msg
)

add_message_files(
  DIRECTORY
  behaviorController/msg
  FILES
  faceDetectionData.msg 
  overtAttentionMode.msg
)

add_message_files(
  DIRECTORY
  person_detection/msg
  FILES
  person_detection_msg_file.msg
)

# Services
add_service_files(
  DIRECTORY
  animateBehaviour/srv
  FILES
  setActivation.srv
)

add_service_files(
  DIRECTORY
  gestureExecution/srv
  FILES
  performGesture.srv
)

add_service_files(
  DIRECTORY
  overtAttention/srv
  FILES
  setMode.srv
)

add_service_files(
  DIRECTORY 
  text_to_speech/srv
  FILES
  TTS.srv
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  behaviorController/srv
  FILES
  animateBehaviorSetActivation.srv
  gestureExecutionPerformGesture.srv
  overtAttentionSetMode.srv
  robotLocalizationResetPose.srv
  robotNavigationSetGoal.srv
  speechEventSetEnabled.srv
  speechEventSetLanguage.srv
  tabletEventPromptAndGetResponse.srv
  textToSpeechSayText.srv
)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  # INCLUDE_DIRS include
  INCLUDE_DIRS behaviorController gestureExecution overtAttention animateBehaviour
  # LIBRARIES 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge geometry_msgs message_runtime utilities behaviortree_cpp
)

find_package(OpenCV REQUIRED)
    catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
behaviorController/include
animateBehaviour/include
overAttention/include
gestureExecution/include
  ${catkin_INCLUDE_DIRS}
)
add_subdirectory(animateBehaviour)
add_subdirectory(gestureExecution)
add_subdirectory(overtAttention)
add_subdirectory(behaviorController)

