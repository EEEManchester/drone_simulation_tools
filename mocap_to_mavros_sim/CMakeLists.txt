cmake_minimum_required(VERSION 3.0.2)
project(mocap_to_mavros_sim)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	qualisys
	geometry_msgs
)



catkin_package(
	LIBRARIES 
	CATKIN_DEPENDS qualisys
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(mocap_to_mavros_sim_node src/mocap_to_drone_node.cpp)
target_link_libraries(mocap_to_mavros_sim_node
  ${catkin_LIBRARIES}
)
