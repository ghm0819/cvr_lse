cmake_minimum_required(VERSION 2.8.3)
project(cvr_lse)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

### for visualization
add_definitions(-DGround)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	rospy
  	cv_bridge
  	# pcl library
  	pcl_conversions
  	# msgs
  	std_msgs
  	sensor_msgs
  	geometry_msgs
  	nav_msgs
  	message_generation
)

find_package(OpenCV REQUIRED
	COMPONENTS
	opencv_highgui
	CONFIG
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_message_files(
	DIRECTORY msg
	FILES
	cloud_label.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  	CATKIN_DEPENDS
)

include_directories(
	main
	ground_detecttion
	grid_map
		cvr_lse_process
	${catkin_INCLUDE_DIRS}
)

set(GRID_SOURCE_FILE
		grid_map/iterators/CircleIterator.cpp
		grid_map/iterators/EllipseIterator.cpp
		grid_map/iterators/GridMapIterator.cpp
		grid_map/iterators/LineIterator.cpp
		grid_map/iterators/PolygonIterator.cpp
		grid_map/iterators/SpiralIterator.cpp
		grid_map/iterators/SubmapIterator.cpp
		grid_map/BufferRegion.cpp
		grid_map/CubicInterpolation.cpp
		grid_map/GridMap.cpp
		grid_map/GridMapMath.cpp
		grid_map/Polygon.cpp
		grid_map/SubmapGeometry.cpp)

set(GROUND_SOURCE_FILE
		ground_detecttion/ground_segmentation.cc
		ground_detecttion/segment.cc
		ground_detecttion/bin.cc)

set(VR_LSE_SOURCE_FILE
		cvr_lse_process/scan_point.cpp
		cvr_lse_process/odometry.cpp
		cvr_lse_process/cvr_lse_process.cpp
		cvr_lse_process/post_process.cpp
		cvr_lse_process/concave_polygon.cpp)

add_executable(cvr_lse main/entrance_process.cpp ${GROUND_SOURCE_FILE} ${GRID_SOURCE_FILE}
		${VR_LSE_SOURCE_FILE})
add_executable(ground_segmentation main/entrance_ground.cpp ${GROUND_SOURCE_FILE})

target_link_libraries(cvr_lse ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(ground_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

