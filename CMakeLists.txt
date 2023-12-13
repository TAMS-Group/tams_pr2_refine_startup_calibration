cmake_minimum_required(VERSION 3.0.2)
project(tams_pr2_refine_startup_calibration)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pluginlib
  pr2_controller_interface
  roscpp
)

add_service_files(FILES
  SetZeroOffset.srv
  GetZeroOffset.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/set_zero_offset_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES controller_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
