cmake_minimum_required(VERSION 3.2)

project(dess)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS program_options REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
		pcl_ros
		cv_bridge
		position_controller
		quadrotor_common
		dodgelib
		eigen_catkin
		quadrotor_msgs
		dodgeros_msgs)
catkin_package(
	CATKIN_DEPENDS
)

# submodules sources
add_subdirectory(modules/ruckig)

# planner sources
include_directories(modules/ruckig/include include ${catkin_INCLUDE_DIRS})
add_subdirectory(src)

add_executable(main main.cpp)
target_link_libraries(main
					  dess
					  RootFinder
					  ruckig
					  ${OpenCV_LIBS}
					  ${catkin_LIBRARIES}
                      Boost::program_options)
