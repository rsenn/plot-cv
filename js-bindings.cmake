

# include: OpenCV
find_package(OpenCV REQUIRED)



function(make_shared_module FNAME)
  string(REGEX REPLACE "_" "-" NAME "${FNAME}")
  string(TOUPPER "${FNAME}" UNAME)

     message("Module: ${NAME}")

    add_library(quickjs-${NAME} SHARED src/js_${FNAME}.cpp src/jsbindings.cpp src/js.cpp)

    target_link_libraries(quickjs-${NAME} ${OpenCV_LIBS} quickjs dl)
    set_target_properties(quickjs-${NAME} PROPERTIES
        PREFIX ""
        BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
#        COMPILE_FLAGS "-fvisibility=hidden"
        BUILD_RPATH "${CMAKE_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR};${CMAKE_BINARY_DIR}/quickjs;${CMAKE_CURRENT_BINARY_DIR}/quickjs"
    )
    target_compile_definitions(quickjs-${NAME} PRIVATE -DJS_${UNAME}_MODULE=1)
    install(TARGETS  quickjs-${NAME} DESTINATION bin)

if(OpenCV_FOUND)
    target_include_directories(quickjs-${NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(quickjs-${NAME} ${OpenCV_LIBS})
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
#add_dependencies(quickjs-contour quickjs-mat quickjs-rect quickjs-point)

add_dependencies(quickjs-contour quickjs-mat)

target_link_libraries(quickjs-point-iterator quickjs-contour quickjs-mat)
#add_dependencies(quickjs-point-iterator quickjs-contour quickjs-mat)

file(GLOB JS_BINDINGS_SOURCES 
    src/color.cpp
    src/data.cpp
    src/geometry.cpp
    #src/js.cpp
    src/jsbindings.cpp
    #src/plot-cv.cpp
    src/js_*.cpp
    src/js.cpp
    src/line.cpp
    src/matrix.cpp
    src/polygon.cpp
    src/*.h 
    src/*.hpp 
)


# Main
add_library(quickjs-opencv MODULE  ${JS_BINDINGS_SOURCES} )
set_target_properties(quickjs-opencv PROPERTIES
  #  COMPILE_FLAGS "-fvisibility=hidden"
    PREFIX ""
)
target_compile_definitions(quickjs-opencv PRIVATE "-DJS_BINDINGS_INIT_MODULE=1")
target_link_libraries(quickjs-opencv ${OpenCV_LIBS} quickjs-shared)
# link
