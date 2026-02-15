#[[cmake_minimum_required(VERSION 3.9)
project(imgui-viewer)
set(CMAKE_CXX_STANDARD 14)
]]

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindGLFW.cmake)
# include: OpenCV

# glfw
if(NOT PKG_CONFIG_FOUND)
  include(FindPkgConfig)
endif(NOT PKG_CONFIG_FOUND)
pkg_check_modules(GLFW3 glfw3 REQUIRED)
#[[

find_package(glfw3 REQUIRED)]]
include_directories(${GLFW_INCLUDE_DIRS})
link_libraries(${GLFW_LIBRARY_DIRS})

set(GLFW_LIBRARIES ${GLFW_LIBRARY} ${pkgcfg_lib_GLFW3_glfw})
# opengl
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# glew
pkg_check_modules(GLEW glew REQUIRED)

set(GLEW_LIBRARIES ${pkgcfg_lib_GLEW_GL} ${pkgcfg_lib_GLEW_GLEW} ${pkgcfg_lib_GLEW_GLU})
# find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

if(APPLE)
  find_library(COCOA_LIBRARY Cocoa)
  find_library(OpenGL_LIBRARY OpenGL)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(COREVIDEO_LIBRARY CoreVideo)
  set(EXTRA_LIBS ${COCOA_LIBRARY} ${OpenGL_LIBRARY} ${IOKIT_LIBRARY} ${COREVIDEO_LIBRARY})
endif(APPLE)

if(WIN32)
  # nothing now
endif(WIN32)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2-config.cmake)

if(ANDROID)
  add_definitions(-DIMGUI_IMPL_OPENGL_ES2=1)
else(ANDROID)
  find_package(GLEW)

  add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLEW=1 -DIMGUI_DEBUG_LOG=1 -DIMGUI_IMPL_OPENGL_LOADER_GLEW=1)
endif(ANDROID)

file(GLOB IMGUI_VIEWER_SOURCES src/color.cpp src/data.cpp src/geometry.cpp src/js.cpp src/jsbindings.cpp
     src/plot-cv.cpp src/line.cpp src/matrix.cpp src/polygon.cpp src/util.cpp src/*.h src/*.hpp qjs-opencv/js_*.cpp)

set(QUICKJS_SOURCES ${QUICKJS_SOURCES})

add_definitions(-D_GNU_SOURCE=1)

include_directories(imgui imgui/backends)
include_directories(qjs-opencv qjs-opencv/include)

# Main
add_executable(
  imgui-viewer
  src/imgui-viewer.cpp imgui/imgui.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp
  #imgui/libs/gl3w/GL/gl3w.c
  ${IMGUI_VIEWER_SOURCES})
target_compile_definitions(imgui-viewer PRIVATE #CONFIG_VERSION="${QUICKJS_VERSION}"
                                                CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM=1 ${PLOTCV_DEFS})

# link
target_link_libraries(
  imgui-viewer ${SDL2_LIBRARIES} ${QUICKJS_OPENCV_A} ${PNG_LIBRARIES} ${OPENCV_FREETYPE_LIBRARY} ${OPENCV_LIBRARIES}
  glfw ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} ${EXTRA_LIBS} quickjs)

if(OpenCV_FOUND)
  target_include_directories(imgui-viewer PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(imgui-viewer ${OpenCV_LIBS})
endif()

install(TARGETS imgui-viewer DESTINATION bin)

add_dependencies(imgui-viewer qjs-opencv)
