cmake_minimum_required (VERSION 2.8)
project (plot-cv)

find_package(OpenCV 3.1 REQUIRED)


include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

add_definitions(-std=c++11)

macro(add_prog name)

  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} ${OpenCV_LIBS})

  install(TARGETS ${name} DESTINATION bin )
endmacro()

add_prog(CannyWebcam1)
add_prog(findLaserPoint)
add_prog(mouth)
#add_prog(eyebrow)


add_prog(App)
add_prog(canny)
add_prog(contour)
add_prog(Contour_detection)
add_prog(contours2)
add_prog(contours3)
add_prog(contours4)
add_prog(contours)
add_prog(distance)
add_prog(images)
add_prog(laser)
add_prog(mulobject)
#7u add_prog(MusicJamboree)
add_prog(o_angle)
#add_prog(ObjectProcessor)
add_prog(SimplerBlobDetector)
add_prog(Source)
add_prog(v_point)
#add_prog(v_point_pid)$