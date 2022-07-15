cmake_minimum_required(VERSION 2.8.3)
project(fake_qualisys)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	std_msgs
	qualisys
)
find_package(Eigen3 REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${eigen3_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS qualisys
        DEPENDS EIGEN3
)

add_executable(fake_qualisys_gazebo_module src/fake_qualisys_gazebo_module.cpp)

target_link_libraries(
	fake_qualisys_gazebo_module
	${catkin_LIBRARIES}
)

add_executable(fake_vicon_gazebo_module src/fake_Vicon_gazebo_module.cpp)

target_link_libraries(
	fake_vicon_gazebo_module
	${catkin_LIBRARIES}
)