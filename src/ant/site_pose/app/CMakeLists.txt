cmake_minimum_required(VERSION 2.8.3)
project(app)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  message_generation
  std_msgs
  autoware_msgs
)

find_package(Qt5  REQUIRED COMPONENTS  Widgets  )
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/main.cpp
  src/app.cpp
  src/qrviz.cpp
)

set(FORMS
  src/app.ui
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
   message_runtime   # This will not be the only thing here
   std_msgs
)



file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/app/*.hpp *.h basewidget/*.hpp *.h)

include_directories(
  include/app

  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${SOURCES} ${FORMS} )

target_link_libraries(${PROJECT_NAME}_node
   
   ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node  Qt5::Widgets )


install(TARGETS 
  ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY  launch/ image
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)