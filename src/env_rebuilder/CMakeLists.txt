cmake_minimum_required(VERSION 2.8.3)
project(env_rebuilder)

set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs tf
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

# find PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable(test_SIFTMatcher test/test_sift_matcher.cpp)
#add_dependencies(test_SIFTMatcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_SIFTMatcher ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(test_ORBMatcher test/test_orb_matcher.cpp)
#add_dependencies(test_ORBMatcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_ORBMatcher ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(test_PCLLib test/test_pcl.cpp)
#add_dependencies(test_PCLLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_PCLLib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(testHashMap src/config.cpp src/camera.cpp src/lidarpoint.cpp src/point_cloud.cpp test/testHashMap.cpp )
#add_dependencies(testHashMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(testHashMap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES})

#add_executable(test_Viz test/testViz.cpp)
#add_dependencies(test_Viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_Viz ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(test_LibFeature test/test_LibFeature.cpp)
#add_dependencies(test_LibFeature ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_LibFeature ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Sophus_LIBRARIES} ${PCL_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)
#
#add_executable(test_TriDecode test/test_Tri_decode.cpp)
#add_dependencies(test_TriDecode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_TriDecode ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_simple_rebuild_V3 test/test_simple_rebuild_V3.cpp
#        src/lidarpoint.cpp src/config.cpp src/camera.cpp src/point_cloud.cpp src/feature.cpp src/feature_map.cpp src/frame.cpp src/config.cpp)
#add_dependencies(test_simple_rebuild_V3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_simple_rebuild_V3 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(transfer_viewer_node test/transfer_viewer.cpp)
add_dependencies(transfer_viewer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transfer_viewer_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(template_Tri test/template_Tri.cpp)
add_dependencies(template_Tri ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(template_Tri ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES})

add_executable(pcd_viewer_node test/pcd_viewer.cpp)
add_dependencies(pcd_viewer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_viewer_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(simple_rebuild_V1 src/simple_rebuild_V1.cpp )
add_dependencies(simple_rebuild_V1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_rebuild_V1 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(simple_rebuild_V2 src/simple_rebuild_V2.cpp )
add_dependencies(simple_rebuild_V2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_rebuild_V2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(simple_rebuild_V3 src/simple_rebuild_V3.cpp src/frame.cpp src/point_cloud.cpp src/feature_map.cpp
        src/config.cpp src/camera.cpp  src/feature.cpp src/surface.cpp src/lidarpoint.cpp)
add_dependencies(simple_rebuild_V3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_rebuild_V3 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES})

#add_executable(simple_rebuild_V3 src/simple_rebuild_V3.cpp )
#add_dependencies(simple_rebuild_V2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(simple_rebuild_V2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
# target_link_libraries(xxx ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Sophus_LIBRARIES} ${PCL_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)