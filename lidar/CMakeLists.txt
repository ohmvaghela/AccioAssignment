cmake_minimum_required(VERSION 3.0.2)
project(lidar)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  xacro

)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch urdf world)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


add_executable(depthScan src/depthScan.cpp)
target_link_libraries(depthScan ${catkin_LIBRARIES} )
