cmake_minimum_required(VERSION 2.8)
project(DBoW2)


if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

file(GLOB SBow2 src/DBoW2/*.cpp)
file(GLOB SUtil src/DUtils/*.cpp)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../lib)

add_library(${PROJECT_NAME} SHARED ${SBow2} ${SUtil})
include_directories(include include/DBoW2 include/DUtils)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
