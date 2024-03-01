# Top-level CMakeLists.txt for vn_driver

cmake_minimum_required(VERSION 3.0.2)
project(my_vn_driver)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

# Declare ROS messages, services, and actions
add_message_files(
  FILES
  Vectornav.msg
)

# Generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare ROS dynamic reconfigure parameters
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

# Catkin specific configuration
catkin_package()

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vn_driver.cpp
# )

# Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
# With catkin_make, all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vn_driver_node.cpp)

# Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# Install targets
# all install targets should use catkin DESTINATION variables

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# Testing
# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vn_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
# catkin_add_nosetests(test)

