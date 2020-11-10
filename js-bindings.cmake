# include: OpenCV
include(${CMAKE_CURRENT_SOURCE_DIR}/opencv.cmake)

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

function(make_shared_module FNAME)
  string(REGEX REPLACE "_" "-" NAME "${FNAME}")
  string(TOUPPER "${FNAME}" UNAME)

  message("Module: ${NAME}")
  set(TARGET_NAME quickjs-${NAME})

  add_library(${TARGET_NAME} SHARED src/js_${FNAME}.cpp src/jsbindings.cpp src/js.cpp)

  target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})
  set_target_properties(
    ${TARGET_NAME}
    PROPERTIES PREFIX ""
               BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
               INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/quickjs"
               OUTPUT_NAME "${NAME}"
               # COMPILE_FLAGS "-fvisibility=hidden"
               BUILD_RPATH "${CMAKE_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR};${CMAKE_BINARY_DIR}/quickjs;${CMAKE_CURRENT_BINARY_DIR}/quickjs")
  target_compile_definitions(${TARGET_NAME} PRIVATE -DJS_${UNAME}_MODULE=1 -DCONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}")
  install(TARGETS ${TARGET_NAME} DESTINATION lib/quickjs)

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

message("JS_BINDINGS_MODULES: ${JS_BINDINGS_MODULES}")

foreach(JS_MODULE ${JS_BINDINGS_MODULES})

  make_shared_module(${JS_MODULE})

endforeach()

add_dependencies(quickjs-rect quickjs-point quickjs-size)
# add_dependencies(quickjs-contour quickjs-mat quickjs-rect quickjs-point)

add_dependencies(quickjs-contour quickjs-mat)

target_link_libraries(quickjs-point-iterator quickjs-point)
target_link_libraries(quickjs-contour quickjs-point-iterator)
target_link_libraries(quickjs-line quickjs-point)
target_link_libraries(quickjs-rect quickjs-size quickjs-point)
target_link_libraries(quickjs-video-capture quickjs-mat)
target_link_libraries(quickjs-cv quickjs-mat quickjs-contour quickjs-line)

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
                                                OUTPUT_NAME "opencv" PREFIX "")
target_compile_definitions(quickjs-opencv PRIVATE -DJS_BINDINGS_INIT_MODULE=1 -DCONFIG_PREFIX=\"${CMAKE_INSTALL_PREFIX}\")

target_link_libraries(quickjs-opencv ${OpenCV_LIBS})
# link
