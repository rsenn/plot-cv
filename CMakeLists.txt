cmake_minimum_required (VERSION 2.8)
project (plot-cv)

include(CheckLibraryExists)
check_library_exists(json-cpp ZN4YAML6detail9node_data9reset_mapEv /usr JSON_CPP)


find_package(OpenCV 3.1 REQUIRED)


include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

add_definitions(-std=c++11)

macro(add_prog)

  set(ARGS ${ARGN})
  list(GET ARGS 0 name)
  list(REMOVE_AT ARGS 0)

  add_executable(${name} ${name}.cpp ${ARGS})
  target_link_libraries(${name} ${OpenCV_LIBS})

  install(TARGETS ${name} DESTINATION bin )
endmacro()

add_prog(CannyWebcam1)
# add_prog(findLaserPoint)
add_prog(mouth)
#add_prog(eyebrow)


#add_prog(App)
add_prog(canny)
add_prog(contour)
add_prog(Contour_detection)
add_prog(contours2)
#add_prog(contours3)
add_prog(contours4)
add_prog(contours)
add_prog(distance)
add_prog(images polygon.cpp data.cpp)
add_prog(laser)
#add_prog(mulobject)
#7u add_prog(MusicJamboree)
add_prog(o_angle)
#add_prog(ObjectProcessor)
add_prog(Source)
add_prog(v_point)
#add_prog(v_point_pid)$
#add_prog(BGMain_Raspberry)
#add_prog(cameranavigator)
add_prog(code6-1)
add_prog(code6-2)
#add_prog(code6-4)
#add_prog(code6-5)
add_prog(code7-3)
add_prog(connected_components)
#add_prog(counter)
add_prog(findContours_demo)
add_prog(fitellipse)
add_prog(generalContours_demo1)
add_prog(generalContours_demo2)
add_prog(hull_demo)
add_prog(imageSegmentation)
add_prog(imgproc_drawContours)
add_prog(introduction_to_pca)
add_prog(moments_demo)
add_prog(motionTracking)
add_prog(motionTracking_modified)
add_prog(obj_track)
add_prog(pointPolygonTest_demo)
#add_prog(qt_opengl)
add_prog(segment_objects)
add_prog(shape_example)
add_prog(SimplerBlobDetector)
add_prog(squares)
#add_prog(tracking_calibration)
add_prog(tracking_calibration_with_img)
target_link_libraries(tracking_calibration_with_img yaml-cpp roslib rospack roscpp )
add_prog(video)
target_link_libraries(video roslib rospack rosconsole roscpp roscpp_serialization)
add_prog(watershed)
