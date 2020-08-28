cmake_minimum_required(VERSION 2.8.3)
project(pose_estimator)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_msgs
  geometry_msgs
  tf
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs tf2_msgs geometry_msgs tf
)


include_directories(
        include ${catkin_INCLUDE_DIRS}
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimator_node src/pose_estimatorV1.cpp )
add_dependencies(pose_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_estimator_node ${catkin_LIBRARIES})

add_executable(pose_estimator_cleanbot_node src/pose_estimator_cleanbot.cpp )
add_dependencies(pose_estimator_cleanbot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_estimator_cleanbot_node ${catkin_LIBRARIES} ${OpenCV_LIBS})