cmake_minimum_required(VERSION 3.0.2)
project(moving_objects)

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
install(PROGRAMS
  scripts/box_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch world urdf config)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


add_executable(box_controller_node src/box_controller_node.cpp)
target_link_libraries(box_controller_node ${catkin_LIBRARIES} )
