cmake_minimum_required(VERSION 2.8.3)
project(asr_aruco_marker_recognition)

# C++11 is needed
add_definitions(-std=gnu++0x -ggdb)
set(CMAKE_BUILD_TYPE Debug)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    add_definitions(-DOPENCV_VERSION_3=1)
endif()

#find_package(OpenCV 2 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
  message_filters
  cv_bridge
  camera_calibration_parsers
  asr_ros_uri
  asr_ivt_bridge
  asr_ivt
  message_generation
  asr_object_database
  asr_msgs
)

generate_dynamic_reconfigure_options( cfg/ArucoMarkerRecognition.cfg)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_service_files(
    FILES
    ReleaseRecognizer.srv
    GetRecognizer.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs message_filters cv_bridge camera_calibration_parsers asr_ros_uri asr_ivt_bridge asr_ivt asr_object_database asr_msgs dynamic_reconfigure
    DEPENDS OpenCV
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

FILE(GLOB_RECURSE LIBFILES "include/*.h")
set(ARUCO_SRC include/aruco/ar_omp.cpp
                    include/aruco/arucofidmarkers.cpp
                    include/aruco/board.cpp
                    include/aruco/boarddetector.cpp
                    include/aruco/cameraparameters.cpp
                    include/aruco/chromaticmask.cpp
                    include/aruco/cvdrawingutils.cpp
                    include/aruco/highlyreliablemarkers.cpp
                    include/aruco/marker.cpp
                    include/aruco/markerdetector.cpp
                    include/aruco/subpixelcorner.cpp)

add_executable(asr_aruco_marker_recognition
                    src/aruco_marker_recognition.cpp
                    src/marker_detection.cpp
                    ${ARUCO_SRC}
                    ${LIBFILES})

add_dependencies(asr_aruco_marker_recognition
        ${PROJECT_NAME}_gencfg
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

TARGET_LINK_LIBRARIES(asr_aruco_marker_recognition
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
)

add_executable(marker_creator
                    src/marker_creator.cpp
                    ${ARUCO_SRC}
                    ${LIBFILES})

TARGET_LINK_LIBRARIES(marker_creator
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
)


