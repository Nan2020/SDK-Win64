cmake_minimum_required (VERSION 2.8)

project(TestIndem)

#set include
FILE(GLOB_RECURSE TEST_HEADER_FILES include/*.h *.h ../../include)
file(GLOB_RECURSE TEST_SOURCE_FILES Indem.cpp)
set(HEADER_DIR include)
include_directories(${TEST_HEADER_FILES} ${CMAKE_SOURCE_DIR}/../../include)

option(DISPLAY_POINT_CLOUD "build point cloud display window" OFF)
if(${DISPLAY_POINT_CLOUD})
add_definitions(-DDISPLAY_POINT_CLOUD)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
endif()

#generate dll
add_executable(${PROJECT_NAME} ${TEST_SOURCE_FILES} ${TEST_HEADER_FILES})
#target_compile_options(TestIndem PUBLIC "/MT")

#set link libraries
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/../../indem.lib )
if(${DISPLAY_POINT_CLOUD})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()