cmake_minimum_required (VERSION 2.8)

project(seethrough)

FILE(GLOB_RECURSE HEADER_FILES include/*.h src/*.h include/*.hpp)
FILE(GLOB_RECURSE SOURCE_FILES src/*.cpp)

include_directories(include ${OpenCV_DIR}/include
  ${CMAKE_SOURCE_DIR}/../../include )

add_definitions(-DWINVER=0x0500)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
set_target_properties(${PROJECT_NAME}  PROPERTIES FOLDER "plugin_seethrough")

target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBS})

# add_subdirectory(test)