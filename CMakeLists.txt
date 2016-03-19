cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)

SET(QT_DONT_USE_QTGUI TRUE)
INCLUDE(${QT_USE_FILE})

file(GLOB "./tools/include")
include_directories(./tools/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Qt4_INCLUDE_DIRS})
#file(tool_srcs ${CMAKE_SOURCE_DIR}/tools/*.cpp)

add_executable(VideoShow ./tools/ParameterSetter/VideoShow.cpp)
add_executable(ParameterSetter ./tools/ParameterSetter/ParameterSetter.cpp)

target_link_libraries(VideoShow
	${OpenCV_LIBRARIES}
)

target_link_libraries(ParameterSetter 
	${OpenCV_LIBRARIES}
	${Qt4_LIBRARIES}
)

