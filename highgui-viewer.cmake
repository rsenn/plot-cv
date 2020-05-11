


set(GLEW_USE_STATIC_LIBS TRUE)
set(GLEW_VERBOSE TRUE)

file(GLOB HIGHGUI_VIEWER_SOURCES 
    src/color.cpp
    src/data.cpp
    src/geometry.cpp
    src/js.cpp

    src/jsbindings.cpp

    src/plot-cv.cpp

    src/js_contour.cpp src/js_draw.cpp src/js_mat.cpp src/js_point.cpp src/js_rect.cpp src/js_size.cpp

    src/line.cpp
    src/matrix.cpp
    src/polygon.cpp
    src/*.h 
    src/*.hpp 
)
add_executable(highgui-viewer src/highgui-viewer.cpp ${HIGHGUI_VIEWER_SOURCES})
target_link_libraries(highgui-viewer ${OpenCV_LIBS} ${GLEW_STATIC_LIBRARIES} quickjs-static ${ELECTRICFENCE_LIBRARY})

install(TARGETS highgui-viewer DESTINATION bin)
