cmake_minimum_required(VERSION 2.8.3)
project(mbf_gridmap_nav)


find_package(catkin REQUIRED
        COMPONENTS
        actionlib
        actionlib_msgs
        dynamic_reconfigure
        geometry_msgs
        grid_map_core
        grid_map_msgs
        grid_map_ros
        mbf_abstract_nav
        mbf_gridmap_core
        mbf_msgs
        mbf_abstract_core
        mbf_utility
        nav_msgs
        roscpp
        std_msgs
        std_srvs
        tf
        pluginlib
)

find_package(Boost COMPONENTS thread chrono REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
        cfg/MoveBaseFlex.cfg
)

set(MBF_GRIDMAP_LIB mbf_gridmap_server)
set(MBF_GRIDMAP_NODE mbf_gridmap_nav)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${MBF_GRIDMAP_LIB}
        CATKIN_DEPENDS
        actionlib
        actionlib_msgs
        dynamic_reconfigure
        geometry_msgs
        mbf_abstract_nav
        grid_map_core
        grid_map_msgs
        grid_map_ros
        mbf_gridmap_core
        mbf_msgs
        mbf_abstract_core
        mbf_utility
        nav_msgs
        pluginlib
        roscpp
        std_msgs
        std_srvs
        tf
        pluginlib
        DEPENDS Boost
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_library(${MBF_GRIDMAP_LIB}
        src/gridmap_navigation_server.cpp
        src/gridmap_planner_execution.cpp
        src/gridmap_controller_execution.cpp
        src/gridmap_recovery_execution.cpp
)

add_dependencies(${MBF_GRIDMAP_LIB} ${PROJECT_NAME}_gencfg)
add_dependencies(${MBF_GRIDMAP_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_GRIDMAP_LIB}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_executable(${MBF_GRIDMAP_NODE} src/gridmap_node.cpp)
add_dependencies(${MBF_GRIDMAP_NODE} ${MBF_GRIDMAP_LIB})
target_link_libraries(${MBF_GRIDMAP_NODE}
        ${MBF_GRIDMAP_LIB}
        ${catkin_LIBRARIES}
)

install(TARGETS
        ${MBF_GRIDMAP_LIB} ${MBF_GRIDMAP_NODE}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
