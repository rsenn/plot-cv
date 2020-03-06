
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
target_link_libraries(highgui-viewer ${OpenCV_LIBS} quickjs-static ${ELECTRICFENCE_LIBRARY})

