cmake_minimum_required(VERSION 2.8.3)
project(driverless_ui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  message_generation
  std_msgs
)

find_package(Qt5 COMPONENTS Core Concurrent REQUIRED)
find_package(Qt5  REQUIRED COMPONENTS  Widgets  )
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_message_files(
  FILES
  HardwareInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime   # This will not be the only thing here
  std_msgs
)

#${CMAKE_CURRENT_SOURCE_DIR}这是当前处理的CMakeLists.txt所在的目录,可以理解为一个绝对路径。
# set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/include)
#link_directories(${PROJECT_SOURCE_DIR}/lib)   #链接第三方包所在路径

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/driverless.cpp
  src/qrviz.cpp
  src/driverless.ui
  ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP}
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES INSTALL_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so
)

target_link_libraries(${PROJECT_NAME}_node  Qt5::Widgets Qt5::Core Qt5::Concurrent)

install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}    #程序运行的时候调用install中的库路径
)

install(DIRECTORY
  include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY launch config lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
