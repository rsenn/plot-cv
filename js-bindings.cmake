

# include: OpenCV
find_package(OpenCV REQUIRED)


function(make_shared_module NAME)
     message("Module: ${NAME}")

    add_library(quickjs-${NAME} MODULE src/js_${NAME}.cpp src/jsbindings.cpp src/js.cpp)

  string(TOUPPER "${NAME}" UNAME)
    target_link_libraries(quickjs-${NAME} ${OpenCV_LIBS} quickjs dl)
    set_target_properties(quickjs-${NAME} PROPERTIES
        PREFIX ""
        COMPILE_FLAGS "-fvisibility=hidden"
    )
    target_compile_definitions(quickjs-${NAME} PRIVATE -DJS_${UNAME}_MODULE=1)
    install(TARGETS  quickjs-${NAME} DESTINATION bin)

if(OpenCV_FOUND)
    target_include_directories(quickjs-${NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(quickjs-${NAME} ${OpenCV_LIBS})
endif()
endfunction()

file(GLOB JS_BINDINGS_SOURCES ${CMAKE_SOURCE_DIR}/src/js_*.cpp)
list(APPEND JS_BINDINGS_MODULES ${JS_BINDINGS_SOURCES})

  list(TRANSFORM JS_BINDINGS_MODULES REPLACE  "\\.cpp$" "")
  list(TRANSFORM JS_BINDINGS_MODULES REPLACE  ".*/js_" "")

  message("JS_BINDINGS_MODULES: ${JS_BINDINGS_MODULES}")




foreach(JS_MODULE ${JS_BINDINGS_MODULES})

    make_shared_module(${JS_MODULE})

endforeach()

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
add_library(quickjs-opencv MODULE 

    ${JS_BINDINGS_SOURCES}
)

# link
