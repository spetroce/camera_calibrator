cmake_minimum_required(VERSION 2.8.11)
project(CameraCalibrator)

## User defined library/include paths
include(PkgConfigPath.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${MIO_INCLUDE_DIR}/mio/cmake/Modules")

## Setup Release and Debug variables
include(${MIO_INCLUDE_DIR}/mio/cmake/DefaultConfigTypes.cmake)

## mio
include_directories(${MIO_INCLUDE_DIR})

## OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(OPENCV_LIBRARIES ${OpenCV_LIBS})
message(STATUS "Found OpenCV version ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")

## Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 5 REQUIRED COMPONENTS Widgets Core OpenGL Svg Concurrent PrintSupport Xml)

## camera_calibrator
qt5_wrap_ui(camera_calibrator_UI camera_calibrator.ui)
qt5_add_resources(camera_calibrator_RESOURCES resources/qt_icons.qrc)
add_executable(camera_calibrator camera_calibrator.cpp ${camera_calibrator_UI} ${camera_calibrator_RESOURCES})
target_link_libraries(camera_calibrator Qt5::Widgets ${OPENCV_LIBRARIES} X11)

if(EXISTS "other")
  add_executable(opencv_calibration other/calibration.cpp)
  target_link_libraries(opencv_calibration ${OPENCV_LIBRARIES})

  add_executable(opencv_img_list_creator other/image_list_creator.cpp)
  target_link_libraries(opencv_img_list_creator ${OPENCV_LIBRARIES})
endif()
