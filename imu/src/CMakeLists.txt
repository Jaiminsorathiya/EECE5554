# Toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(MyProject)

set(MY_CATKIN_TOPLEVEL TRUE)

# Search for catkin within the workspace
set(_cmd "my_catkin_find_pkg" "my_catkin" "${CMAKE_SOURCE_DIR}")
execute_process(COMMAND ${_cmd}
  RESULT_VARIABLE _res
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
if(NOT _res EQUAL 0 AND NOT _res EQUAL 2)
  # Searching for my_catkin resulted in an error
  string(REPLACE ";" " " _cmd_str "${_cmd}")
  message(FATAL_ERROR "Search for 'my_catkin' in workspace failed (${_cmd_str}): ${_err}")
endif()

# Include my_catkin from workspace or via find_package()
if(_res EQUAL 0)
  set(my_catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/${_out}/cmake")
  # Include all.cmake without add_subdirectory to let it operate in the same scope
  include(${my_catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  add_subdirectory("${_out}")

else()
  # Use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
  # or CMAKE_PREFIX_PATH from the environment
  if(NOT DEFINED MY_CMAKE_PREFIX_PATH)
    if(NOT "$ENV{MY_CMAKE_PREFIX_PATH}" STREQUAL "")
      if(NOT WIN32)
        string(REPLACE ":" ";" MY_CMAKE_PREFIX_PATH $ENV{MY_CMAKE_PREFIX_PATH})
      else()
        set(MY_CMAKE_PREFIX_PATH $ENV{MY_CMAKE_PREFIX_PATH})
      endif()
    endif()
  endif()

  # List of my_catkin workspaces
  set(my_catkin_search_path "")
  foreach(path ${MY_CMAKE_PREFIX_PATH})
    if(EXISTS "${path}/.my_catkin")
      list(FIND my_catkin_search_path ${path} _index)
      if(_index EQUAL -1)
        list(APPEND my_catkin_search_path ${path})
      endif()
    endif()
  endforeach()

  # Search for my_catkin in all workspaces
  set(MY_CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(my_catkin QUIET
    NO_POLICY_SCOPE
    PATHS ${my_catkin_search_path}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(MY_CATKIN_TOPLEVEL_FIND_PACKAGE)

  if(NOT my_catkin_FOUND)
    message(FATAL_ERROR "find_package(my_catkin) failed. my_catkin was neither found in the workspace nor in the MY_CMAKE_PREFIX_PATH. One reason may be that no setup.sh was sourced before.")
  endif()
endif()

my_catkin_workspace()

