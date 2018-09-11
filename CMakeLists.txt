cmake_minimum_required(VERSION 2.8.11)
project(CameraCalibrator)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")

## User defined library/include paths
include(PkgConfigPath.cmake)

## Setup Release and Debug variables
include("${CMAKE_CURRENT_LIST_DIR}/modules/mio/cmake/DefaultConfigTypes.cmake")

## mio
include_directories("${CMAKE_CURRENT_LIST_DIR}/modules")

## OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(OPENCV_LIBRARIES ${OpenCV_LIBS})
message(STATUS "Found OpenCV version ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")

## Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 5 REQUIRED COMPONENTS Widgets Core OpenGL Svg Concurrent PrintSupport Xml)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/modules/camera_calibrator")

