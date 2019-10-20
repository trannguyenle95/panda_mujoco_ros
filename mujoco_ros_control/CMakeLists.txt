cmake_minimum_required(VERSION 2.8.3)
project(mujoco_ros_control)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-std=c++11)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp roslib hardware_interface controller_manager cv_bridge image_transport std_msgs message_generation)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

add_service_files(
        FILES
        ChangePose.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${mujoco_INCLUDE_DIR} ${glfw3_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIRS})

add_executable(mujoco_control src/mujoco_control.cpp src/RobotHWMujoco.cpp src/RenderImage.cpp)
target_link_libraries(mujoco_control ${mujoco_LIBRARIES} ${OpenCV_LIBS} ${glfw3_LIBRARIES} ${libglew_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_gl_LIBRARY})
add_dependencies(mujoco_control ${catkin_EXPORTED_TARGETS} mujoco_ros_control_gencpp)

install(TARGETS mujoco_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

if (CATKIN_ENABLE_TESTING)
    add_subdirectory(tests)
endif ()
