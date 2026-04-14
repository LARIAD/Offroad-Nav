# ##############################################################################
# Setup version
file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/VERSION version)
message(STATUS "Read version from file: ${version}")

cmake_minimum_required(VERSION 3.5.1)
project(virtual_costmap_layer LANGUAGES C CXX VERSION ${version})

set(CMAKE_CXX_STANDARD 14)
set(CXX_STANDARD_REQUIRED ON)

# ##############################################################################
# Load and setup packages

find_package(catkin
             REQUIRED
             std_msgs
             roscpp
             costmap_2d
             dynamic_reconfigure
             barakuda_manager
             tf
             tf2)

# ##############################################################################
# Generate messages and configurations
generate_dynamic_reconfigure_options(cfg/VirtualLayer.cfg)

# ##############################################################################
# Setup catkin packages
catkin_package(INCLUDE_DIRS include
               LIBRARIES virtual_layer
               CATKIN_DEPENDS std_msgs costmap_2d dynamic_reconfigure tf tf2
               DEPENDS)

# ##############################################################################
# Generate library
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(virtual_layer src/virtual_layer.cpp)
target_link_libraries(virtual_layer ${catkin_LIBRARIES})
add_dependencies(virtual_layer ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
