cmake_minimum_required(VERSION 3.0.2)
project(cssr_system)

## ──────────────────────────────────────────────────────────────────────────────
## 1) Find catkin and all ROS‐level components that robotLocalization needs
##
##    (We list ALL ROS packages that get linked into ${catkin_LIBRARIES}.)
## ──────────────────────────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs          # ← this must appear in package.xml
  geometry_msgs     # ← also here
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
  actionlib
  control_msgs
  message_generation
)

## ──────────────────────────────────────────────────────────────────────────────
## 2) Find any external (non‐ROS) libraries
## ──────────────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS aruco)
find_package(yaml-cpp REQUIRED)

## ──────────────────────────────────────────────────────────────────────────────
## 3) Tell CMake where to find your service definitions
##
##    We keep them under robotLocalization/srv/, so downstream generation works.
## ──────────────────────────────────────────────────────────────────────────────
add_service_files(
  DIRECTORY robotLocalization/srv
  FILES
    ResetPose.srv
    SetPose.srv
)

## ──────────────────────────────────────────────────────────────────────────────
## 4) Generate the C++ headers for those services
## ──────────────────────────────────────────────────────────────────────────────
generate_messages(
  DEPENDENCIES
    std_msgs   # (only if your .srv files reference any std_msgs types)
)

## ──────────────────────────────────────────────────────────────────────────────
## 5) Declare this cssr_system “meta‐package”
##
##    • Every ROS component listed above must appear in CATKIN_DEPENDS.
##    • We add message_runtime so that downstream code (robotLocalization) sees
##      the generated <cssr_system/ResetPose.h> and <cssr_system/SetPose.h>.
##    • OpenCV & yaml-cpp remain in DEPENDS (external, non‐catkin libraries).
## ──────────────────────────────────────────────────────────────────────────────
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    image_transport
    tf2_ros
    tf2_geometry_msgs
    actionlib
    control_msgs
    message_runtime
  DEPENDS
    OpenCV
    yaml-cpp
)

## ──────────────────────────────────────────────────────────────────────────────
## 6) Export include paths so that any subfolder sees ROS/OpenCV/YAML‐CPP headers
## ──────────────────────────────────────────────────────────────────────────────
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## ──────────────────────────────────────────────────────────────────────────────
## 7) Finally, descend into robotLocalization/ (leaving its CMake exactly as‐is)
## ──────────────────────────────────────────────────────────────────────────────
add_subdirectory(robotLocalization)
