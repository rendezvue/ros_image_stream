project(rosrun_prog)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#opencv
find_package(OpenCV REQUIRED)
include_directories( ${OPENCV_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

#src
set(SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src)
FILE(GLOB SRC 
	"${SRC_PATH}/*.cpp" 
)
include_directories(${SRC_PATH})

add_executable( rosrun_prog 
	${SRC} 
)

target_link_libraries(rosrun_prog ${OpenCV_LIBS})
