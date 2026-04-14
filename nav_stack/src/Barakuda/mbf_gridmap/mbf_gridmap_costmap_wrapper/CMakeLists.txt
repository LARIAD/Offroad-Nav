cmake_minimum_required(VERSION 3.0.2)
project(mbf_gridmap_costmap_wrapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        mbf_gridmap_core
        mbf_costmap_nav
        tf2_ros
        costmap_2d
        roscpp
)

generate_dynamic_reconfigure_options(
        configs/MBGridmapLayer.cfg
)

find_package(Boost COMPONENTS thread chrono REQUIRED)

set(MBF_GRIDMAP_COSTMAP_WRAPPER mbf_gridmap_costmap_wrapper)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES mbf_gridmap_costmap_wrapper
        CATKIN_DEPENDS mbf_gridmap_core roscpp tf2_ros costmap_2d dynamic_reconfigure mbf_costmap_nav
        DEPENDS Boost
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${MBF_GRIDMAP_COSTMAP_WRAPPER}
        plugins/wrapper_local_planner.cpp
        plugins/wrapper_global_planner.cpp
        plugins/movebase_gridmap_layer.cpp
)
add_dependencies(${MBF_GRIDMAP_COSTMAP_WRAPPER} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_GRIDMAP_COSTMAP_WRAPPER}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

install(TARGETS
        ${MBF_GRIDMAP_COSTMAP_WRAPPER}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES wrapper_global_planner.xml wrapper_local_planner.xml mb_gridmap_layer.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        DIRECTORY launch configs
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
