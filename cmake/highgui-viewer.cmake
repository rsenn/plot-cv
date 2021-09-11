set(GLEW_USE_STATIC_LIBS TRUE)
set(GLEW_VERBOSE TRUE)

file(GLOB HIGHGUI_VIEWER_SOURCES src/color.cpp src/data.cpp src/js.cpp src/plot-cv.cpp src/matrix.cpp src/polygon.cpp src/*.h)

set(QUICKJS_SOURCES ${QUICKJS_SOURCES})

add_definitions(-D_GNU_SOURCE=1)

add_executable(highgui-viewer src/highgui-viewer.cpp ${HIGHGUI_VIEWER_SOURCES})
target_compile_definitions(highgui-viewer PRIVATE _GNU_SOURCE=1 CONFIG_VERSION="${QUICKJS_VERSION}" CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM=1 ${PLOTCV_DEFS})

target_link_libraries(highgui-viewer ${QUICKJS_OPENCV_A} ${OpenCV_LIBS} opencv_freetype ${GLEW_SHARED_LIBRARY_RELEASE} ${GLEW_SHARED_LIBRARIES} quickjs png ${LIBDL} ${LIBM} ${LIBPTHREAD})

add_dependencies(highgui-viewer qjs-opencv)
install(TARGETS highgui-viewer DESTINATION bin)
