cmake_minimum_required(VERSION 2.8.3)
project(arm_scan_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
  tf
)

add_message_files(
        FILES
        Scanpackage.msg
)

add_service_files(
        FILES
        ScanPoseXYZ.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime cv_bridge tf
)

include_directories(
        include ${catkin_INCLUDE_DIRS}
        include
        ${catkin_INCLUDE_DIRS}
)

# find eigen3
include_directories( "/usr/include/eigen3")

# find Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

# find g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS} "/usr/include/eigen3")

# find Ceres
find_package(Ceres REQUIRED PATHS)
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

# find Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(tf_listener_node test/tf_listener.cpp )
add_dependencies(tf_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_listener_node ${catkin_LIBRARIES})

add_executable(test_scan_controllerV2_node test/test_scan_controllerV2.cpp )
add_dependencies(test_scan_controllerV2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_scan_controllerV2_node ${catkin_LIBRARIES})

add_executable(scan_controllerV1_node src/scan_controllerV1.cpp )
add_dependencies(scan_controllerV1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_controllerV1_node ${catkin_LIBRARIES})

add_executable(scan_controllerV2_node src/scan_controllerV2.cpp )
add_dependencies(scan_controllerV2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_controllerV2_node ${catkin_LIBRARIES})

add_executable(scan_controllerV3_node src/scan_controllerV3.cpp )
add_dependencies(scan_controllerV3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_controllerV3_node ${catkin_LIBRARIES})



