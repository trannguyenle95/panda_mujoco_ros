cmake_minimum_required(VERSION 2.8.3)
project(panda_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS xacro
        moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface controller_interface
        hardware_interface kdl_parser)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(
        CATKIN_DEPENDS controller_interface hardware_interface
        INCLUDE_DIRS include
        LIBRARIES pandacontrollers)

add_library(pandacontrollers src/ForceController_KDL.cpp src/HybridController_KDL.cpp)

target_link_libraries(pandacontrollers ${catkin_LIBRARIES})

add_executable(run src/run.cpp src/Robot.cpp src/ForceController_KDL.cpp src/HybridController_KDL.cpp)
target_link_libraries(run ${catkin_LIBRARIES})
add_dependencies(run ${catkin_EXPORTED_TARGETS})

add_executable(parser src/parser.cpp)
target_link_libraries(parser ${catkin_LIBRARIES})

file(GLOB xacro_files model/robots/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)

install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES panda_controllers_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS pandacontrollers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS run
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
