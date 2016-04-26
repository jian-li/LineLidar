cmake_minimum_required(VERSION 2.8)

project(2dslam)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
        pcl_conversions
	cv_bridge
        tf
        serial
	sensor_msgs
	laser_geometry
)

#set the cmake module paths
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules
                      ${CMAKE_MODULE_PATH})

find_package(libpointmatcher REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
find_package(PCL REQUIRED)
find_package(YamlCpp REQUIRED)

SET(QT_DONT_USE_QTGUI TRUE)
INCLUDE(${QT_USE_FILE})

catkin_package(
	INCLUDE_DIRS 
)

# includ all the header 
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
# 
# file(GLOB "./tools/include")
# file(GLOB "./include")

include_directories(./tools/include)
include_directories(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(YAMLCPP_INCLUDE_DIR)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(
	./include
	${Qt4_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)



add_executable(VideoShow ./tools/ParameterSetter/VideoShow.cpp)
add_executable(ParameterSetter ./tools/ParameterSetter/ParameterSetter.cpp)
add_executable(NeotoLidarDriver src/neotolidardriver.cpp)
add_executable(2dslam src/2dslam.cpp)
add_executable(camera_setting src/camera_setting.cpp)

add_executable(LineLidarNode
	./src/TriangulationNode.cpp
)

add_executable(pcl_icp src/pcl_icp.cpp)

target_link_libraries(VideoShow
	${OpenCV_LIBRARIES}
)

target_link_libraries(ParameterSetter 
	${OpenCV_LIBRARIES}
	${Qt4_LIBRARIES}
)

target_link_libraries(LineLidarNode
	${catkin_LIBRARIES}
)

target_link_libraries(NeotoLidarDriver
	${catkin_LIBRARIES}
)

target_link_libraries(camera_setting
    ${OpenCV_LIBRARIES}
    )
    
target_link_libraries(2dslam
    ${catkin_LIBRARIES}
    ${libpointmatcher_LIBRARIES}
    ${YAMLCPP_LIBRARY}
)

target_link_libraries(pcl_icp 
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    )

# add_subdirectory(ekf_attitude)