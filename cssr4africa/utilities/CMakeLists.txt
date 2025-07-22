cmake_minimum_required(VERSION 2.8.3)
project(utilities)

## Compile as C++11
add_compile_options(-std=c++11)

## Find Catkin and any catkin components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

## Declare this a Catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utilities_lib
  CATKIN_DEPENDS roscpp roslib
)

## Include paths
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## 1) Build the core utilities library
add_library(utilities_lib
  src/behaviorControllerUtilities.cpp                 
  src/cultureKnowledgeBaseImplementation.cpp
  src/environmentKnowledgeBaseImplementation.cpp
)
target_link_libraries(utilities_lib
  ${catkin_LIBRARIES}
)

## 2) cultureKnowledgeBase example
add_executable(cultureKnowledgeBaseExample
  src/cultureKnowledgeBaseApplication.cpp
)
set_target_properties(cultureKnowledgeBaseExample PROPERTIES
  PREFIX ""
  OUTPUT_NAME cultureKnowledgeBaseExample
)
target_link_libraries(cultureKnowledgeBaseExample
  utilities_lib
  ${catkin_LIBRARIES}
)
add_dependencies(cultureKnowledgeBaseExample
  ${catkin_EXPORTED_TARGETS}
)

## 3) environmentKnowledgeBase example
add_executable(environmentKnowledgeBaseExample
  src/environmentKnowledgeBaseApplication.cpp
)
set_target_properties(environmentKnowledgeBaseExample PROPERTIES
  PREFIX ""
  OUTPUT_NAME environmentKnowledgeBaseExample
)
target_link_libraries(environmentKnowledgeBaseExample
  utilities_lib
  ${catkin_LIBRARIES}
)
add_dependencies(environmentKnowledgeBaseExample
  ${catkin_EXPORTED_TARGETS}
)
