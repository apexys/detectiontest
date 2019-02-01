cmake_minimum_required (VERSION 2.8.11)
cmake_policy(SET CMP0043 NEW)

set(MAIN_NAME detectiontest)

project ( ${MAIN_NAME} CXX )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(TENSORFLOW "extern/tensorflow") 

find_package( OpenCV REQUIRED ) 
include_directories( ${OpenCV_INCLUDE_DIRS} ${TENSORFLOW} )

set( SOURCE
   ${SOURCE}
   src/main.cpp
   src/marker.cpp
   src/markerlist.cpp
   src/neural.cpp
)
set( HEADERS
   ${HEADERS}
   src/markerlist.h
   src/marker.h
   src/neural.h
)

LINK_DIRECTORIES(${TENSORFLOW})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_executable( ${MAIN_NAME} ${SOURCE} ${HEADERS} ${UI_HEADERS} )

target_link_libraries( ${MAIN_NAME} ${OpenCV_LIBS} tensorflow)
