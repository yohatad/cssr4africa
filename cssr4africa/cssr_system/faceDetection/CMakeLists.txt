cmake_minimum_required(VERSION 3.5.1)
project(face_detection)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
  image_transport
  std_msgs
  message_generation
)
add_message_files(
  DIRECTORY msg
  FILES faceDetectionData.msg
  # Message1.msg
  # Message2.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

# find_package(OpenCV 3 REQUIRED)
#     catkin_package(
#     CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
# )

find_package(OpenCV REQUIRED)
    catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
)

if (OpenCV_FOUND)
    # If the package has been found, several variables will
    # be set, you can find the full list with descriptions
    # in the OpenCVConfig.cmake file.
    # Print some message showing some of them
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else ()
    message(FATAL_ERROR "Could not locate OpenCV")
endif()

find_package(Boost COMPONENTS program_options filesystem REQUIRED)

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
 INCLUDE_DIRS include
 LIBRARIES face_detection
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
# file(GLOB SOURCE_FILES src/*.cpp)

# add_executable        (${PROJECT_NAME}_faceDetection src/faceDetection.cpp)
# set_target_properties (${PROJECT_NAME}_faceDetection PROPERTIES OUTPUT_NAME faceDetection PREFIX "")
# target_link_libraries (${PROJECT_NAME}_faceDetection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable        (${PROJECT_NAME}_faceDetectionApplication src/faceDetectionApplication.cpp src/faceDetectionImplementation.cpp)
set_target_properties (${PROJECT_NAME}_faceDetectionApplication PROPERTIES OUTPUT_NAME faceDetectionApplication PREFIX "")
target_link_libraries (${PROJECT_NAME}_faceDetectionApplication ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable        (${PROJECT_NAME}_faceDetectionStub src/faceDetectionStub.cpp)
set_target_properties (${PROJECT_NAME}_faceDetectionStub PROPERTIES OUTPUT_NAME faceDetectionStub PREFIX "")
target_link_libraries (${PROJECT_NAME}_faceDetectionStub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable        (${PROJECT_NAME}_faceDetectionDriver src/faceDetectionDriver.cpp)
set_target_properties (${PROJECT_NAME}_faceDetectionDriver PROPERTIES OUTPUT_NAME faceDetectionDriver PREFIX "")
target_link_libraries (${PROJECT_NAME}_faceDetectionDriver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable        (${PROJECT_NAME}_test src/test.cpp src/faceDetectionImplementation.cpp)
# set_target_properties (${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME test PREFIX "")
# target_link_libraries (${PROJECT_NAME}_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

