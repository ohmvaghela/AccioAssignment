cmake_minimum_required(VERSION 3.0.2)
project(lidar_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)
catkin_package()

add_message_files(
  FILES
  Obstacles.msg
  )

  

foreach(dir msg)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

