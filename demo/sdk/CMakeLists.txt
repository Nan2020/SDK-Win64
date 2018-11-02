cmake_minimum_required (VERSION 2.8)

project(TestIndem)

#set include
FILE(GLOB_RECURSE TEST_HEADER_FILES include/*.h *.h)
file(GLOB_RECURSE TEST_SOURCE_FILES *.cpp)
set(HEADER_DIR include)
include_directories(${TEST_HEADER_FILES})

#generate dll
add_executable(${PROJECT_NAME} ${TEST_SOURCE_FILES} ${TEST_HEADER_FILES})
#target_compile_options(TestIndem PUBLIC "/MT")

#set link libraries
if(WIN32)
target_link_libraries(${PROJECT_NAME}
   indem)
else(WIN32)
target_link_libraries(${PROJECT_NAME} gtest indem ${SLAM_LIB} tbb 
	cholmod
	boost_system boost_filesystem
	protobuf protobuf-lite nanomsg stdc++fs)
endif(WIN32)