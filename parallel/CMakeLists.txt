cmake_minimum_required(VERSION 2.8)
project(VoLoop)

ADD_DEFINITIONS(-std=c++11 -DGLOG_NO_ABBREVIATED_SEVERITIES)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


find_package(OpenCV REQUIRED)
message("version is ${OPENCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
FIND_PACKAGE( G2O REQUIRED )
FIND_PACKAGE( Eigen3 REQUIRED )

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )


include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/DBoW2/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/DLib/include)

file(GLOB D_Lib "${PROJECT_SOURCE_DIR}/ThirdParty/lib/libDLib.*")
file(GLOB DB_Lib "${PROJECT_SOURCE_DIR}/ThirdParty/lib/libDBoW2.*")

set(DLib_LIBS ${D_Lib})
set(DBoW2_LIBS ${DB_Lib})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(main src/main.cc src/vo.cc src/GraphOptimization.cc src/util.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${DLib_LIBS} ${DBoW2_LIBS}
g2o_core g2o_types_slam3d g2o_solver_csparse g2o_solver_cholmod g2o_stuff g2o_csparse_extension g2o_types_sba ${CSPARSE_LIBRARY})


###Test files
