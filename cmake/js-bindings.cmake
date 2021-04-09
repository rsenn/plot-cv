# include: OpenCV
include(${CMAKE_SOURCE_DIR}/cmake/opencv.cmake)

if(WIN32 OR MINGW)
  set(QUICKJS_LIBRARY_DIR "${quickjs_BINARY_DIR}")
  set(QUICKJS_MODULE_DEPENDENCIES "quickjs")
  set(QUICKJS_MODULE_CFLAGS "-fvisibility=hidden")
endif(WIN32 OR MINGW)
function(config_shared_module TARGET_NAME)
  if(QUICKJS_LIBRARY_DIR)
    target_link_directories(${TARGET_NAME} PRIVATE "${QUICKJS_LIBRARY_DIR}")
  endif(QUICKJS_LIBRARY_DIR)
  if(QUICKJS_MODULE_DEPENDENCIES)
    target_link_libraries(${TARGET_NAME} ${QUICKJS_MODULE_DEPENDENCIES})
  endif(QUICKJS_MODULE_DEPENDENCIES)
  if(QUICKJS_MODULE_CFLAGS)
    target_compile_options(${TARGET_NAME} PRIVATE "${QUICKJS_MODULE_CFLAGS}")
  endif(QUICKJS_MODULE_CFLAGS)
endfunction(config_shared_module TARGET_NAME)

set(JS_BINDINGS_COMMON src/color.hpp src/geometry.hpp src/js.hpp src/js_alloc.hpp src/js_array.hpp src/js_contour.hpp src/js_line.hpp src/js_point.hpp src/js_rect.hpp src/js_size.hpp src/js_typed_array.hpp src/jsbindings.hpp src/plot-cv.hpp src/psimpl.hpp src/util.hpp)
set(js_line_SOURCES src/line.cpp src/line.hpp)

function(make_shared_module FNAME)
  string(REGEX REPLACE "_" "-" NAME "${FNAME}")
  string(TOUPPER "${FNAME}" UNAME)

  set(TARGET_NAME quickjs-${NAME})

  add_library(${TARGET_NAME} SHARED src/js_${FNAME}.cpp ${js_${FNAME}_SOURCES} src/jsbindings.cpp src/util.cpp src/js.hpp src/js.cpp ${JS_BINDINGS_COMMON})

  target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})
  set_target_properties(
    ${TARGET_NAME}
    PROPERTIES PREFIX "" # BUILD_RPATH "${OPENCV_LIBRARY_DIRS}:${CMAKE_CURRENT_BINARY_DIR}"
               RPATH "${OPENCV_LIBRARY_DIRS}:${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/quickjs" OUTPUT_NAME "${NAME}" # COMPILE_FLAGS "-fvisibility=hidden"
               BUILD_RPATH "${CMAKE_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_BINARY_DIR}/quickjs:${CMAKE_CURRENT_BINARY_DIR}/quickjs")
  target_compile_definitions(${TARGET_NAME} PRIVATE JS_${UNAME}_MODULE=1 CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" ${PLOTCV_DEFS})
  install(TARGETS ${TARGET_NAME} DESTINATION lib/quickjs PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

  config_shared_module(${TARGET_NAME})

  if(OpenCV_FOUND)
    target_include_directories(${TARGET_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})
  endif()
endfunction()

file(GLOB JS_BINDINGS_SOURCES ${CMAKE_SOURCE_DIR}/src/js_*.cpp)

foreach(MOD ${JS_BINDINGS_SOURCES})
  string(REGEX REPLACE "\\.cpp" "" MOD "${MOD}")
  string(REGEX REPLACE ".*/js_" "" MOD "${MOD}")
  list(APPEND JS_BINDINGS_MODULES ${MOD})

endforeach(MOD ${JS_BINDINGS_SOURCES})


foreach(JS_MODULE ${JS_BINDINGS_MODULES})

  make_shared_module(${JS_MODULE})

endforeach()

string(REPLACE ";" " " MODULE_NAMES "${JS_BINDINGS_MODULES}")
message(STATUS "Configured modules: ${MODULE_NAMES}")


add_dependencies(quickjs-rect quickjs-point quickjs-size)
# add_dependencies(quickjs-contour quickjs-mat quickjs-rect quickjs-point)

add_dependencies(quickjs-contour quickjs-mat)

target_link_libraries(quickjs-mat quickjs-size)
target_link_libraries(quickjs-point-iterator quickjs-line quickjs-point)
target_link_libraries(quickjs-contour quickjs-point-iterator quickjs-mat)
target_link_libraries(quickjs-line quickjs-point)
target_link_libraries(quickjs-rect quickjs-size quickjs-point)
target_link_libraries(quickjs-video-capture quickjs-mat)
target_link_libraries(quickjs-cv quickjs-mat quickjs-contour quickjs-rect quickjs-line)
target_link_libraries(quickjs-draw quickjs-mat quickjs-contour quickjs-size)
target_link_libraries(quickjs-clahe quickjs-mat quickjs-size)
target_link_libraries(quickjs-umat quickjs-mat)
target_link_libraries(quickjs-subdiv2d quickjs-contour)

# add_dependencies(quickjs-point-iterator quickjs-contour quickjs-mat)

file(
  GLOB
  JS_BINDINGS_SOURCES
  src/color.cpp
  src/data.cpp
  src/geometry.cpp
  # src/js.cpp
  src/jsbindings.cpp
  # src/plot-cv.cpp
  src/js_*.cpp
  src/js.cpp
  src/line.cpp
  src/matrix.cpp
  src/polygon.cpp
  src/*.h
  src/*.hpp)

# Main
add_library(quickjs-opencv MODULE ${JS_BINDINGS_SOURCES})
config_shared_module(quickjs-opencv)

set_target_properties(quickjs-opencv PROPERTIES # COMPILE_FLAGS "-fvisibility=hidden"
                                                RPATH "${OPENCV_LIBRARY_DIRS}:${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/quickjs" OUTPUT_NAME "opencv" PREFIX "")
target_compile_definitions(quickjs-opencv PRIVATE -DJS_BINDINGS_INIT_MODULE=1 -DCONFIG_PREFIX=\"${CMAKE_INSTALL_PREFIX}\" ${PLOTCV_DEFS})

target_link_libraries(quickjs-opencv ${OpenCV_LIBS})
# link
