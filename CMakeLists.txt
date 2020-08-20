cmake_minimum_required(VERSION 2.8.3)
project(topic_publisher)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)
find_package(PythonLibs REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/BoxParams.cfg
  cfg/RelativePoseParams.cfg
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)
