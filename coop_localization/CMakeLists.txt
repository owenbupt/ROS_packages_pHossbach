cmake_minimum_required(VERSION 2.8.3)
project(coop_localization)
SET(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  std_msgs
  urdf
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
##generate_messages(
##   DEPENDENCIES
##   std_msgs  # Or other packages containing msgs
##)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES coop_localization
   CATKIN_DEPENDS roscpp rospy tf geometry_msgs std_msgs urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(coop_localization
#   src/${PROJECT_NAME}/coop_localization.cpp
# )


set(C2C_LEFT_SOURCE_FILES
    src/coop_c2c/c2c_left.cpp
    src/coop_c2c/c2c_left_node.cpp
)

set(C2C_RIGHT_SOURCE_FILES
    src/coop_c2c/c2c_right.cpp
    src/coop_c2c/c2c_right_node.cpp
)

set(C2C_MERGE_SOURCE_FILES
    src/coop_c2c/c2c_merge.cpp
    src/coop_c2c/c2c_merge_node.cpp
)

set(FOLLOW_ROTATION_SOURCE_FILES
    src/coop_follow/follow_rotation.cpp
    src/coop_follow/follow_rotation_node.cpp
)

set(RECORD_WEBCAM2_SOURCE_FILES
    src/coop_record/record_webcam2.cpp
    src/coop_record/record_webcam2_node.cpp
)

set(RECORD_LIBVISO2_SOURCE_FILES
    src/coop_record/record_libviso2.cpp
    src/coop_record/record_libviso2_node.cpp
)

set(RECORD_C2C_SOURCE_FILES
    src/coop_record/record_c2c.cpp
    src/coop_record/record_c2c_node.cpp
)

set(RECORD_FUSION_SOURCE_FILES
    src/coop_record/record_fusion.cpp
    src/coop_record/record_fusion_node.cpp
)

set(COOP_FUSION_SOURCE_FILES
    src/coop_fusion/coop_fusion.cpp
    src/coop_fusion/coop_fusion_node.cpp
)



set(C2C_LEFT_HEADER_FILES
    include/c2c_left_node.hpp
)

set(C2C_RIGHT_HEADER_FILES
    include/c2c_right_node.hpp
)

set(C2C_MERGE_HEADER_FILES
    include/c2c_merge_node.hpp
)

set(FOLLOW_ROTATION_HEADER_FILES
    include/follow_rotation_node.hpp
)

set(RECORD_WEBCAM2_HEADER_FILES
    include/record_webcam2_node.hpp
)

set(RECORD_LIBVISO2_HEADER_FILES
    include/record_libviso2_node.hpp
)

set(RECORD_C2C_HEADER_FILES
    include/record_c2c_node.hpp
)

set(RECORD_FUSION_HEADER_FILES
    include/record_fusion_node.hpp
)

set(COOP_FUSION_HEADER_FILES
    include/coop_fusion_node.hpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(coop_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(coop_localization_node src/coop_localization_node.cpp)

add_executable(c2c_left ${C2C_LEFT_SOURCE_FILES} ${C2C_LEFT_HEADER_FILES})

add_executable(c2c_right ${C2C_RIGHT_SOURCE_FILES} ${C2C_RIGHT_HEADER_FILES})

add_executable(c2c_merge ${C2C_MERGE_SOURCE_FILES} ${C2C_MERGE_HEADER_FILES})

add_executable(follow_rotation ${FOLLOW_ROTATION_SOURCE_FILES} ${FOLLOW_ROTATION_HEADER_FILES})

add_executable(record_webcam2 ${RECORD_WEBCAM2_SOURCE_FILES} ${RECORD_WEBCAM2_HEADER_FILES})

add_executable(record_libviso2 ${RECORD_LIBVISO2_SOURCE_FILES} ${RECORD_LIBVISO2_HEADER_FILES})

add_executable(record_c2c ${RECORD_C2C_SOURCE_FILES} ${RECORD_C2C_HEADER_FILES})

add_executable(record_fusion ${RECORD_FUSION_SOURCE_FILES} ${RECORD_FUSION_HEADER_FILES})

add_executable(coop_fusion ${COOP_FUSION_SOURCE_FILES} ${COOP_FUSION_HEADER_FILES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(coop_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(coop_localization_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(c2c_left ${catkin_LIBRARIES})

target_link_libraries(c2c_right ${catkin_LIBRARIES})

target_link_libraries(c2c_merge ${catkin_LIBRARIES})

target_link_libraries(follow_rotation ${catkin_LIBRARIES})

target_link_libraries(record_webcam2 ${catkin_LIBRARIES})

target_link_libraries(record_libviso2 ${catkin_LIBRARIES})

target_link_libraries(record_c2c ${catkin_LIBRARIES})

target_link_libraries(record_fusion ${catkin_LIBRARIES})

target_link_libraries(coop_fusion ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS coop_localization coop_localization_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_coop_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
