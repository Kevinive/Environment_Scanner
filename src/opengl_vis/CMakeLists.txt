cmake_minimum_required(VERSION 2.8.3)
project(opengl_vis)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)

include_directories(
        include ${catkin_INCLUDE_DIRS}
        include
        ${catkin_INCLUDE_DIRS}
)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

# find Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

# Find GLFW
find_package(glfw3 REQUIRED)

#add_executable(test_OpenGL test/testOpenGL.cpp src/glad.c)
#add_dependencies(test_OpenGL ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_OpenGL ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )
#
#add_executable(test_Tri test/testDrawTri.cpp src/glad.c)
#add_dependencies(test_Tri ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_Tri ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )
#
#add_executable(test_Texture test/testTexture.cpp src/glad.c src/stb_image.cpp)
#add_dependencies(test_Texture ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_Texture ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )
#
#add_executable(test_GLM test/testGLM.cpp src/glad.c src/stb_image.cpp)
#add_dependencies(test_GLM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_GLM ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )
#
#add_executable(test_Camera test/testCamera.cpp src/glad.c src/stb_image.cpp)
#add_dependencies(test_Camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_Camera ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )

add_executable(openGL_showPointCloud src/show_pointcloud.cpp src/glad.c src/stb_image.cpp src/config.cpp src/opengl_viewer.cpp)
add_dependencies(openGL_showPointCloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(openGL_showPointCloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )

add_executable(openGL_texture src/show_texture.cpp src/glad.c src/stb_image.cpp src/config.cpp src/opengl_viewerV2.cpp)
add_dependencies(openGL_texture ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(openGL_texture ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} dl glfw )