


set(GLEW_USE_STATIC_LIBS TRUE)
set(GLEW_VERBOSE TRUE)


file(GLOB PLOT_CV_SOURCES 
    src/color.cpp
    src/data.cpp
    src/geometry.cpp
    src/js.cpp
    src/jsbindings.cpp
    src/line.cpp
    src/matrix.cpp
    src/polygon.cpp
    src/plot-cv.cpp
    src/*.h 
    src/*.hpp 
)
add_executable(highgui-viewer src/highgui-viewer.cpp ${PLOT_CV_SOURCES})
target_link_libraries(highgui-viewer ${OpenCV_LIBS} ${GLEW_STATIC_LIBRARIES} quickjs-static ${ELECTRICFENCE_LIBRARY})

install(TARGETS highgui-viewer DESTINATION bin)
