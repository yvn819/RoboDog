cmake_minimum_required(VERSION 2.8.3)
project(controller_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  mav_msgs
  eigen_conversions
  tf_conversions
  pcl_ros
  octomap_msgs
  visualization_msgs

)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs
  mav_msgs 
  tf
  eigen_conversions
  tf_conversions
  pcl_ros
  octomap_msgs
  visualization_msgs

)

find_package(Eigen3)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



add_executable(controller_node src/controller_node.cpp)
add_executable(test_controller src/test_controller.cpp)
add_executable(pathplanning_node src/pathplanning_node.cpp)
add_executable(visualization src/visualization.cpp)
add_executable(filter_point_cloud src/filter_point_cloud.cpp)

add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(test_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(pathplanning_node ${catkin_EXPORTED_TARGETS})
add_dependencies(visualization ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_point_cloud ${catkin_EXPORTED_TARGETS})



target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(controller_node octomap)

target_link_libraries(test_controller ${catkin_LIBRARIES})
target_link_libraries(test_controller octomap)

target_link_libraries(pathplanning_node ${catkin_LIBRARIES})
target_link_libraries(visualization ${catkin_LIBRARIES})

target_link_libraries(filter_point_cloud ${catkin_LIBRARIES})
target_link_libraries(filter_point_cloud octomap)

# target_link_libraries(pathplanning_node octomap)
