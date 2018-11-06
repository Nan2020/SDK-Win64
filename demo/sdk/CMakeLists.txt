cmake_minimum_required (VERSION 2.8)

project(TestIndem)

#set include
FILE(GLOB_RECURSE TEST_HEADER_FILES include/*.h *.h ../../include)
file(GLOB_RECURSE TEST_SOURCE_FILES Indem.cpp)
set(HEADER_DIR include)
include_directories(${TEST_HEADER_FILES} ${CMAKE_SOURCE_DIR}/../../include)

#generate dll
add_executable(${PROJECT_NAME} ${TEST_SOURCE_FILES} ${TEST_HEADER_FILES})
#target_compile_options(TestIndem PUBLIC "/MT")

#set link libraries
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/../../indem.lib)