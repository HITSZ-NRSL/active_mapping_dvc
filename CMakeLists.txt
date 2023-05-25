cmake_minimum_required(VERSION 3.8)

cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_STANDARD 17)
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl -lbfd")
project(active_mapping LANGUAGES CXX)

######################### Packages #########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_filters
  tf
  tf_conversions
  message_generation
  octomap_msgs
  octomap_ros
  actionlib
  actionlib_msgs
  lkh_tsp_solver
  backward_ros
  costmap_2d
  base_local_planner
  eigen_conversions
  livox_ros_driver
  geometry_msgs
  visualization_msgs
  tf2
  genmsg 
  dynamic_reconfigure
  sensor_msgs	
  nav_msgs
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS serialization system filesystem thread program_options date_time regex timer chrono)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Glog REQUIRED)
find_package(cmake_modules REQUIRED)

########################### Action ################################
add_action_files(
  DIRECTORY action
  FILES SensorAngleUpdate.action 
)

########################### Service ################################
add_service_files(
  FILES
  disp_ray.srv
  insert_point_cloud.srv
  get_world_representation.srv
  save_octomap.srv
  aster_query.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  octomap_msgs
  actionlib_msgs
  nav_msgs 
  visualization_msgs 
  actionlib_msgs 
)

catkin_package(
 INCLUDE_DIRS include  ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} 
 LIBRARIES active_mapping  ${OCTOMAP_LIBRARIES} ${PROJECT_NAME}_world_representation  ${PROJECT_NAME}_utils 
 CATKIN_DEPENDS  message_runtime roscpp geometry_msgs visualization_msgs tf rospy std_msgs lkh_tsp_solver  
)

set(Eigen_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(
include
include/active_mapping
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###########
## libs ##
###########

add_library(${PROJECT_NAME}_movements
  src/active_mapping/movements/geometry_pose.cpp)
target_link_libraries(${PROJECT_NAME}_movements
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_utils
  src/active_mapping/utils/octomap_utils.cpp
  src/active_mapping/utils/rviz_utils.cpp
  src/active_mapping/utils/tictoc.cpp
  src/active_mapping/utils/bounding_box.cpp
  src/active_mapping/utils/math_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_utils
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_controller
  src/active_mapping/controller/gazebo_gimbal_controller.cpp
)
target_link_libraries(${PROJECT_NAME}_controller
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
)

add_library(${PROJECT_NAME}_world_representation
  src/active_mapping/world_representation/ig_tree.cpp
  src/active_mapping/world_representation/world_representation.cpp
)

target_link_libraries(${PROJECT_NAME}_world_representation
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}_utils
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_sample_viewpoints
	src/active_mapping/view_space/sample_viewpoints.cpp
)
target_link_libraries(${PROJECT_NAME}_sample_viewpoints
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PROJECT_NAME}_world_representation
  ${PCL_LIBRARIES}
)

add_library(gridastar 
    src/gridastar/AStar2.cpp
    include/gridastar/AStar2.h
    ) 

add_library(${PROJECT_NAME}_view_space
  src/active_mapping/view_space/view.cpp
  src/active_mapping/view_space/frontier_view_space_finder.cpp
  src/active_mapping/view_space/frontier.cpp
)


add_library(${PROJECT_NAME}
  src/active_mapping/active_mapping.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_movements
  ${PROJECT_NAME}_view_space
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_world_representation
  ${PROJECT_NAME}_controller
  ${PROJECT_NAME}_sample_viewpoints
)

#############
##  Apps  ##
#############

########### active_mapping_task #########
add_executable(${PROJECT_NAME}_task
  src/apps/active_mapping_task.cpp
)

target_link_libraries(${PROJECT_NAME}_task
  ${PROJECT_NAME} 
  bfd
)

####### odom_to_tf_node #######
add_executable(odom_to_tf_node
src/apps/odom_to_tf_node.cpp
)
target_link_libraries(odom_to_tf_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
)

####### livox_custom_msg_converter #######
add_executable(livox_custom_msg_converter
  src/apps/livox_custom_msg_converter.cpp
)
target_link_libraries(livox_custom_msg_converter 
  ${catkin_LIBRARIES}
)

install(
  TARGETS   
    ${PROJECT_NAME}_movements
    ${PROJECT_NAME}_view_space
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_world_representation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/active_mapping/world_representation
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
