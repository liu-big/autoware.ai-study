cmake_minimum_required(VERSION 3.0.2)
project(can_client)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(autoware_remove_msgs_DIR "/home/ant/Downloads/install/autoware_remove_msgs/share/autoware_remove_msgs/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  autoware_msgs
  autoware_can_msgs
  autoware_remove_msgs
  autoware_system_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES can_client
#  CATKIN_DEPENDS roscpp rospy std_msgs
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

## Declare a C++ library
add_library(zhongyun
   src/vehicle/zhongyun.cpp 
   src/vehicle/zhongyun_lib/canbase.cpp
)
add_dependencies(zhongyun ${catkin_EXPORTED_TARGETS})
target_link_libraries(zhongyun ${catkin_LIBRARIES})

add_library(wuling
   src/vehicle/wuling.cpp 
   src/vehicle/fuzzy_PID.cpp
   src/vehicle/pid_controller.cpp
   src/vehicle/wuling_lib/canbase.cpp
)
add_dependencies(wuling ${catkin_EXPORTED_TARGETS})
target_link_libraries(wuling ${catkin_LIBRARIES})


add_library(kaiwo
   src/vehicle/kaiwo.cpp 
   src/vehicle/fuzzy_PID.cpp
   src/vehicle/pid_controller.cpp
   src/vehicle/kaiwo_lib/canbase.cpp
)
add_dependencies(kaiwo ${catkin_EXPORTED_TARGETS})
target_link_libraries(kaiwo ${catkin_LIBRARIES})

add_library(yunle
   src/vehicle/yunle.cpp 
   src/vehicle/yunle_lib/canbase.cpp
)
add_dependencies(yunle ${catkin_EXPORTED_TARGETS})
target_link_libraries(yunle ${catkin_LIBRARIES})

add_library(simwuling
   src/vehicle/simwuling.cpp 
)
add_dependencies(simwuling ${catkin_EXPORTED_TARGETS})
target_link_libraries(simwuling ${catkin_LIBRARIES})

add_library(fantai
   src/vehicle/fantai.cpp 
)
add_dependencies(fantai ${catkin_EXPORTED_TARGETS})
target_link_libraries(fantai ${catkin_LIBRARIES})

add_library(yuhsen
   src/vehicle/yuhsen.cpp 
   src/vehicle/yuhsen_lib/canbase.cpp
)
add_dependencies(yuhsen ${catkin_EXPORTED_TARGETS})
target_link_libraries(yuhsen ${catkin_LIBRARIES})

add_library(aian
   src/vehicle/aian.cpp 
   src/vehicle/aian_lib/canbase.cpp
)
add_dependencies(aian ${catkin_EXPORTED_TARGETS})
target_link_libraries(aian ${catkin_LIBRARIES})
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(vehicle_core src/vehicle/vehicle_core.cpp)
add_dependencies(vehicle_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_core ${catkin_LIBRARIES}
  zhongyun
  wuling
  kaiwo
  yunle
  simwuling
  yuhsen
  fantai
  aian
)

add_executable(ultra_bf19
   src/ultrasound/ultra_bf19.cpp
)
add_dependencies(ultra_bf19 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ultra_bf19 ${catkin_LIBRARIES})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_can_client.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

install(TARGETS 
  zhongyun wuling kaiwo yunle simwuling vehicle_core ultra_bf19 yuhsen fantai aian
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY  
  include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

