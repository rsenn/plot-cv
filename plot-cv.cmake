
file(GLOB PLOT_CV_SOURCES 
    src/color.cpp
    src/data.cpp
    src/geometry.cpp
    src/js.cpp
    src/jsbindings.cpp
    src/line.cpp
    src/matrix.cpp
    src/polygon.cpp
    src/*.h 
    src/*.hpp 
)
add_executable(plot-cv src/plot-cv.cpp ${PLOT_CV_SOURCES})
target_link_libraries(plot-cv ${OpenCV_LIBS} quickjs-static ${ELECTRICFENCE_LIBRARY})

