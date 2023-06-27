cmake_minimum_required(VERSION 2.8.3)
project(kimm_polaris3d_smach)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  std_srvs
)

catkin_python_setup()
catkin_package()
