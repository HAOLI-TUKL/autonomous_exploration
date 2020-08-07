cmake_minimum_required(VERSION 2.8.3)
project(my_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_controller
#  CATKIN_DEPENDS controller_interface hardware_interface pluginlib roscpp
#  DEPENDS system_lib
	
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
"/usr/include/gazebo-9/"
"/usr/include/ignition/math4"
"/usr/include/ignition/transport4"
"/usr/include/ignition/msgs1"
"/usr/include/sdformat-6.0"
${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_controller.cpp
# )
add_library(my_controller_lib
  src/my_controller.cpp
)
target_link_libraries(my_controller_lib
  ${GAZEBO_LIBRARIES}
  ${CATKIN_LIBRARIES}
        /opt/ros/melodic/lib/librosbag.so
        /opt/ros/melodic/lib/librosbag_storage.so
        /opt/ros/melodic/lib/librosconsole.so
        /opt/ros/melodic/lib/librosconsole_backend_interface.so
        /opt/ros/melodic/lib/librosconsole_bridge.so
        /opt/ros/melodic/lib/librosconsole_log4cxx.so
        /opt/ros/melodic/lib/libroscpp.so
        /opt/ros/melodic/lib/libroscpp_serialization.so
        /opt/ros/melodic/lib/libroslib.so
        /opt/ros/melodic/lib/libroslz4.so
        /opt/ros/melodic/lib/librospack.so
        /opt/ros/melodic/lib/librostime.so
        /usr/lib/x86_64-linux-gnu/libignition-math4.so
        /usr/lib/x86_64-linux-gnu/libignition-math4.so.4
        /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo.so
        /usr/lib/x86_64-linux-gnu/libgazebo.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_client.so
        /usr/lib/x86_64-linux-gnu/libgazebo_client.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_client.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_common.so
        /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
        /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
        /usr/lib/x86_64-linux-gnu/libgazebo_gui.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_gui.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
        /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
        /usr/lib/x86_64-linux-gnu/libgazebo_ode.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_ode.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
        /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
        /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
        /usr/lib/x86_64-linux-gnu/libgazebo_physics.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_physics.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
        /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
        /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_test_fixture.a
        /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
        /usr/lib/x86_64-linux-gnu/libgazebo_transport.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_transport.so.9.0.0
        /usr/lib/x86_64-linux-gnu/libgazebo_util.so
        /usr/lib/x86_64-linux-gnu/libgazebo_util.so.9
        /usr/lib/x86_64-linux-gnu/libgazebo_util.so.9.0.0
        /opt/ros/melodic/lib/libtf.so
        /opt/ros/melodic/lib/libtf2.so
        /opt/ros/melodic/lib/libtf2_ros.so
        /opt/ros/melodic/lib/libtf_conversions.so
        /opt/ros/melodic/lib/libgazebo_ros_api_plugin.so
        /opt/ros/melodic/lib/libgazebo_ros_block_laser.so
        /opt/ros/melodic/lib/libgazebo_ros_bumper.so
        /opt/ros/melodic/lib/libgazebo_ros_camera.so
        /opt/ros/melodic/lib/libgazebo_ros_camera_utils.so
        /opt/ros/melodic/lib/libgazebo_ros_control.so
        /opt/ros/melodic/lib/libgazebo_ros_depth_camera.so
        /opt/ros/melodic/lib/libgazebo_ros_diff_drive.so
        /opt/ros/melodic/lib/libgazebo_ros_elevator.so
        /opt/ros/melodic/lib/libgazebo_ros_f3d.so
        /opt/ros/melodic/lib/libgazebo_ros_force.so
        /opt/ros/melodic/lib/libgazebo_ros_ft_sensor.so
        /opt/ros/melodic/lib/libgazebo_ros_gpu_laser.so
        /opt/ros/melodic/lib/libgazebo_ros_hand_of_god.so
        /opt/ros/melodic/lib/libgazebo_ros_harness.so
        /opt/ros/melodic/lib/libgazebo_ros_imu.so
        /opt/ros/melodic/lib/libgazebo_ros_imu_sensor.so
        /opt/ros/melodic/lib/libgazebo_ros_joint_pose_trajectory.so
        /opt/ros/melodic/lib/libgazebo_ros_joint_state_publisher.so
        /opt/ros/melodic/lib/libgazebo_ros_laser.so
        /opt/ros/melodic/lib/libgazebo_ros_multicamera.so
        /opt/ros/melodic/lib/libgazebo_ros_openni_kinect.so
        /opt/ros/melodic/lib/libgazebo_ros_p3d.so
        /opt/ros/melodic/lib/libgazebo_ros_paths_plugin.so
        /opt/ros/melodic/lib/libgazebo_ros_planar_move.so
        /opt/ros/melodic/lib/libgazebo_ros_projector.so
        /opt/ros/melodic/lib/libgazebo_ros_prosilica.so
        /opt/ros/melodic/lib/libgazebo_ros_range.so
        /opt/ros/melodic/lib/libgazebo_ros_skid_steer_drive.so
        /opt/ros/melodic/lib/libgazebo_ros_template.so
        /opt/ros/melodic/lib/libgazebo_ros_tricycle_drive.so
        /opt/ros/melodic/lib/libgazebo_ros_triggered_camera.so
        /opt/ros/melodic/lib/libgazebo_ros_triggered_multicamera.so
        /opt/ros/melodic/lib/libgazebo_ros_utils.so
        /opt/ros/melodic/lib/libgazebo_ros_vacuum_gripper.so
        /opt/ros/melodic/lib/libgazebo_ros_video.so



        )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/my_controller_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
