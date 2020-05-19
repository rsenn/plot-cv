#[[cmake_minimum_required(VERSION 3.9)
project(imgui-viewer)
set(CMAKE_CXX_STANDARD 14)
]]

file(GLOB HIGHGUI_VIEWER_SOURCES 
    src/color.cpp
    src/data.cpp
    src/geometry.cpp
    src/js.cpp
    src/jsbindings.cpp
    src/plot-cv.cpp
    src/js_*.cpp
    src/line.cpp
    src/matrix.cpp
    src/polygon.cpp
    src/*.h 
    src/*.hpp 
)
# Main
add_executable(imgui-viewer
    src/imgui-viewer.cpp
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_impl_sdl.cpp
    imgui/imgui_impl_opengl3.cpp
    imgui/libs/gl3w/GL/gl3w.c
    ${HIGHGUI_VIEWER_SOURCES}
)

# include: OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(imgui-viewer PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(imgui-viewer ${OpenCV_LIBS})
endif()

# include: SDL
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

# include: ImGui(symbolic linked)
include_directories(
    imgui/
    imgui/examples
    imgui/libs/gl3w
)

# link
target_link_libraries(imgui-viewer
    ${SDL2_LIBRARIES}
    ${OpenCV_LIBS} ${GLEW_SHARED_LIBRARY_RELEASE} ${GLEW_STATIC_LIBRARY_RELEASE} ${GLEW_LIBRARY_RELEASE} ${GLEW_SHARED_LIBRARY_RELEASE} quickjs ${ELECTRICFENCE_LIBRARY}
    dl
    GL
)

install(TARGETS imgui-viewer DESTINATION bin)
