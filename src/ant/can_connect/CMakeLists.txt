cmake_minimum_required(VERSION 2.8.3)
project(can_connect)

find_package(autoware_build_flags REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS
    autoware_msgs
    autoware_can_msgs
    geometry_msgs
    roscpp
    tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 编译动态库
add_library(can_socket
  src/can_box/can_socket.cpp
)
add_dependencies(can_socket ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_socket ${catkin_LIBRARIES})

add_executable(can_connect
  src/can_connect/can_connect.cpp
  src/can_connect/can_connect_node.cpp
)
add_dependencies(can_connect ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_connect ${catkin_LIBRARIES}
  can_socket            # 添加库
)

install(TARGETS  
  can_socket can_connect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

