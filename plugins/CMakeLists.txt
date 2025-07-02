
# Find gz-cmake and use standard setup
find_package(gz-cmake3 REQUIRED)

# Find dependencies
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-sensors8 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Include dirs
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create the PressureSensor plugin
add_library(PressureSensor SHARED PressureSensor.cc)
target_link_libraries(PressureSensor
  PUBLIC gz-sensors8::gz-sensors8
)

# Create the PressureSensorSystem plugin
add_library(PressureSensorSystem SHARED PressureSensorSystem.cc)
target_link_libraries(PressureSensorSystem
  PUBLIC gz-sim8::gz-sim8
         gz-plugin2::register
         PressureSensor
)

# Install plugins to the plugins/ folder
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/plugins)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/plugins)

# Optional: install targets
install(TARGETS PressureSensor PressureSensorSystem
        LIBRARY DESTINATION lib)

# Optional: if you still use FetchContent for examples (not usually needed)
# FetchContent_Declare(
#   sensors_clone
#   GIT_REPOSITORY https://github.com/gazebosim/gz-sensors
#   GIT_TAG main
# )
# FetchContent_Populate(sensors_clone)
# add_subdirectory(${sensors_clone_SOURCE_DIR}/examples/custom_sensor ${sensors_clone_BINARY_DIR})