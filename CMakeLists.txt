cmake_minimum_required(VERSION 2.8)

project(LineLidar)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
        pcl_conversions
	cv_bridge
        tf
        serial
	sensor_msgs
)

#set the cmake module paths
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules
                      ${CMAKE_MODULE_PATH})

find_package(libpointmatcher REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)

find_package(YamlCpp REQUIRED)

SET(QT_DONT_USE_QTGUI TRUE)
INCLUDE(${QT_USE_FILE})

catkin_package(
	INCLUDE_DIRS 
)


file(GLOB "./tools/include")
file(GLOB "./include")

include_directories(./tools/include)
include_directories(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(YAMLCPP_INCLUDE_DIR)

include_directories(
	./include
	${Qt4_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)



#file(tool_srcs ${CMAKE_SOURCE_DIR}/tools/*.cpp)

add_executable(VideoShow ./tools/ParameterSetter/VideoShow.cpp)
add_executable(ParameterSetter ./tools/ParameterSetter/ParameterSetter.cpp)
add_executable(NeotoLidarDriver src/neotolidardriver.cpp)
add_executable(2dslam src/2dslam.cpp)

add_executable(LineLidarNode
	./src/TriangulationNode.cpp
)

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

target_link_libraries(2dslam
    ${catkin_LIBRARIES}
    ${libpointmatcher_LIBRARIES}
    ${YAMLCPP_LIBRARY}
)
