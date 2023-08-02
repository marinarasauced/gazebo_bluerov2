cmake_minimum_required(VERSION 3.12.2)
project(gazebo_bluerov2)

# add dependencies
include(cmake/CPM.cmake)

find_package(ament_cmake_auto QUIET)
if(${ament_cmake_auto_FOUND})
  message("Building ${PROJECT_NAME} with ament_cmake_auto")
  ament_auto_find_build_dependencies()
  ament_auto_package(INSTALL_TO_SHARE models worlds)
endif()

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
