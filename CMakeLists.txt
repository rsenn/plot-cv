cmake_minimum_required (VERSION 2.8)
project (plot-cv)

find_package(OpenCV 3.1 REQUIRED)


include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

add_definitions(-std=c++11)

add_executable(CannyWebcam1 CannyWebcam1.cpp)
target_link_libraries(CannyWebcam1 ${OpenCV_LIBS})
