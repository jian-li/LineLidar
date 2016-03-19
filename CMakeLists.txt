cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#file(tool_srcs ${CMAKE_SOURCE_DIR}/tools/*.cpp)

add_executable(ParameterSetter ./tools/ParameterSetter/main.cpp)

target_link_libraries(ParameterSetter 
	${OpenCV_LIBRARIES}
)

