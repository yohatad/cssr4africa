add_executable(behaviorController src/behaviorControllerImplementation.cpp src/behaviorControllerApplication.cpp )
set_target_properties(behaviorController PROPERTIES OUTPUT_NAME behaviorController PREFIX "")
target_link_libraries(behaviorController ${catkin_LIBRARIES})
target_include_directories(behaviorController PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(behaviorController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
