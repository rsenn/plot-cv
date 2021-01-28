cmake_minimum_required(VERSION 3.0)
project(plot-cv)

set(DISABLE_WERROR TRUE CACHE BOOL "Disable -Werror flag" FORCE)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "build type")

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)

set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Build with local rpath" FORCE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Build with install rpath" FORCE)

set(quickjs_sources_root ${CMAKE_CURRENT_SOURCE_DIR}/quickjs)
include(${quickjs_sources_root}/defs.cmake)

#[[set(quickjs_version "2020-07-05")
set(quickjs_soversion 1)
]]
set(QUICKJS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Debug")
endif(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")

if(WIN32)
  set(LIBWS2_32 ws2_32)
else(WIN32)

endif(WIN32)

include(FindThreads)
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)
include(CheckTypeSize)
include(FindPkgConfig)

if(ENABLE_CCACHE)
  include("${CMAKE_CURRENT_SOURCE_DIR}/ccache.cmake")
  enable_ccache()
endif(ENABLE_CCACHE)

macro(check_size TYPE VAR)
  check_type_size("${TYPE}" CMAKE_${VAR})
  if(${CMAKE_${VAR}})
    list(APPEND PLOTCV_DEFS ${VAR}=${CMAKE_${VAR}})
  endif(${CMAKE_${VAR}})
  message(STATUS "size of ${TYPE} ${CMAKE_${VAR}}")
endmacro(check_size TYPE VAR)

check_size("long" SIZEOF_LONG)
check_size("long long" SIZEOF_LONG_LONG)
check_size("size_t" SIZEOF_SIZE_T)

if(NOT POSITION_INDEPENDENT_CODE)
  set(POSITION_INDEPENDENT_CODE ON)
endif(NOT POSITION_INDEPENDENT_CODE)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

message("Thread library: ${CMAKE_THREAD_LIBS_INIT}")
if(CMAKE_THREAD_LIBS_INIT)
  set(LIBPTHREAD ${CMAKE_THREAD_LIBS_INIT})
endif(CMAKE_THREAD_LIBS_INIT)

check_library_exists(m atan2 /usr/lib HAVE_LIBM)
if(HAVE_LIBM)
  set(LIBM m)
  # message("Found math library: ${LIBM}")
endif(HAVE_LIBM)

check_library_exists(dl dlopen /usr/lib HAVE_LIBDL)
if(HAVE_LIBDL)
  set(LIBDL dl)
  # message("Found dlfcn library: ${LIBDL}")
endif(HAVE_LIBDL)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib/quickjs")
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_BINARY_DIR}")

check_library_exists(json-cpp ZN4YAML6detail9node_data9reset_mapEv /usr JSON_CPP)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE CACHE BOOL "Build installable binaries with -Wl,-rpath=PREFIX" FORCE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE CACHE BOOL "Add dirs in runtime search path to the install rpath" FORCE)

include(${CMAKE_CURRENT_SOURCE_DIR}/opencv.cmake)

include_directories(${OPENCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/opencv ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/quickjs)

foreach(CXX_STANDARD c++20 c++17 c++14 c++11)
  string(REPLACE "c++" "CPLUSPLUS" CXX_STANDARD_NUM "${CXX_STANDARD}")
  set(CMAKE_REQUIRED_QUIET TRUE)
  check_cxx_compiler_flag("-std=${CXX_STANDARD}" CXX_STANDARD_${CXX_STANDARD_NUM})
  set(CMAKE_REQUIRED_QUIET FALSE)
  if(CXX_STANDARD_${CXX_STANDARD_NUM})
    set(CXX_STANDARD_VALUE "${CXX_STANDARD}" CACHE STRING "C++ standard" FORCE)
    set(CXX_STANDARD_FLAG "-std=${CXX_STANDARD}" CACHE STRING "C++ standard argument" FORCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_STANDARD_FLAG}")
    break()
  endif(CXX_STANDARD_${CXX_STANDARD_NUM})
endforeach()

message("C++ standard: ${CXX_STANDARD_VALUE}")
#[[
check_c_compiler_flag("-Wno-deprecated-anon-enum-enum-conversion" WARN_NO_DEPRECATED_ANON_ENUM_ENUM_CONVERSION)
if(WARN_NO_DEPRECATED_ANON_ENUM_ENUM_CONVERSION)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated-anon-enum-enum-conversion" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-anon-enum-enum-conversion" )
endif(WARN_NO_DEPRECATED_ANON_ENUM_ENUM_CONVERSION)

]]
function(DUMP VAR)
  string(REGEX REPLACE "[;\n]" " " A "${ARGV}")
  message("\nVariable dump of: ${A}\n")

  foreach(VAR ${ARGV})
    message("  ${VAR} = ${${VAR}}")
  endforeach(VAR ${ARGV})
  message("")
endfunction(DUMP VAR)

function(CANONICALIZE OUTPUT_VAR STR)
  string(REGEX REPLACE "^-W" "WARN_" TMP_STR "${STR}")

  string(REGEX REPLACE "-" "_" TMP_STR "${TMP_STR}")
  string(TOUPPER "${TMP_STR}" TMP_STR)

  set("${OUTPUT_VAR}" "${TMP_STR}" PARENT_SCOPE)
endfunction(CANONICALIZE OUTPUT_VAR STR)

macro(NOWARN_FLAG FLAG)
  canonicalize(VARNAME "${FLAG}")
  check_c_compiler_flag("${FLAG}" "${VARNAME}")
  dump(${VARNAME})

  if(${VARNAME})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAG}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")

  endif(${VARNAME})
endmacro(NOWARN_FLAG FLAG)

string(REGEX REPLACE " -Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
string(REGEX REPLACE " -Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

nowarn_flag(-Wno-unused-value)
nowarn_flag(-Wno-unused-variable)

if("${CMAKE_CXX_COMPILER_ID}" MATCHES ".*Clang.*")
  nowarn_flag(-Wno-deprecated-anon-enum-enum-conversion)
  nowarn_flag(-Wno-extern-c-compat)
  nowarn_flag(-Wno-implicit-int-float-conversion)
  nowarn_flag(-Wno-deprecated-enum-enum-conversion)
endif("${CMAKE_CXX_COMPILER_ID}" MATCHES ".*Clang.*")

dump(CMAKE_C_FLAGS CMAKE_CXX_FLAGS)
dump(CMAKE_CXX_FLAGS_DEBUG)

#[[
check_c_compiler_flag("-Wno-unused-value" WARN_NO_UNUSED_VALUE)
if(WARN_NO_UNUSED_VALUE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-value" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-value" )
endif(WARN_NO_UNUSED_VALUE)


check_c_compiler_flag("-Wno-unused-variable" WARN_NO_UNUSED_VARIABLE)
if(WARN_NO_UNUSED_VARIABLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable" )
endif(WARN_NO_UNUSED_VARIABLE)
]]

check_cxx_compiler_flag("-fPIC" F_OPT_PIC)
if(F_OPT_PIC)
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fPIC")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC")
endif(F_OPT_PIC)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")

  message("!!!!DEBUG MODE!!!")
  # option(USE_EFENCE "Enable electric fence" OFF)
  set(DEBUG_OUTPUT TRUE CACHE BOOL "Debug output" FORCE)
  set(DEBUG_MAT FALSE CACHE BOOL "Debug cv::Mat" FORCE)
  list(APPEND PLOTCV_DEFS DEBUG=1)

  # add_definitions(-DDEBUG=1)

  check_c_compiler_flag("-O0" OPT_C_OPT_NONE)
  check_cxx_compiler_flag("-O0" OPT_CXX_OPT_NONE)
  if(OPT_C_OPT_NONE)
    if(NOT "${CMAKE_C_FLAGS_DEBUG}" MATCHES "-O0")
      set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0" CACHE STRING "C compiler options" FORCE)
    endif(NOT "${CMAKE_C_FLAGS_DEBUG}" MATCHES "-O0")
  endif(OPT_C_OPT_NONE)
  if(OPT_CXX_OPT_NONE)
    if(NOT "${CMAKE_CXX_FLAGS_DEBUG}" MATCHES "-O0")
      set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0" CACHE STRING "C++ compiler options" FORCE)
    endif(NOT "${CMAKE_CXX_FLAGS_DEBUG}" MATCHES "-O0")
  endif(OPT_CXX_OPT_NONE)

  check_c_compiler_flag("-ggdb" OPT_C_G_GDB)
  check_cxx_compiler_flag("-ggdb" OPT_CXX_G_GDB)
  if(OPT_C_G_GDB)
    if(NOT "${CMAKE_C_FLAGS_DEBUG}" MATCHES "-ggdb")
      set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ggdb" CACHE STRING "C compiler options" FORCE)
    endif(NOT "${CMAKE_C_FLAGS_DEBUG}" MATCHES "-ggdb")
  endif(OPT_C_G_GDB)
  if(OPT_CXX_G_GDB)
    if(NOT "${CMAKE_CXX_FLAGS_DEBUG}" MATCHES "-ggdb")
      set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb" CACHE STRING "C++ compiler options" FORCE)
    endif(NOT "${CMAKE_CXX_FLAGS_DEBUG}" MATCHES "-ggdb")
  endif(OPT_CXX_G_GDB)

endif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")

if(DEBUG_OUTPUT)
  list(APPEND PLOTCV_DEFS DEBUG_OUTPUT=1)
endif(DEBUG_OUTPUT)
if(DEBUG_MAT)
  list(APPEND PLOTCV_DEFS DEBUG_MAT=1)
endif(DEBUG_MAT)

check_c_compiler_flag("-flto" HAS_F_LTO_FLAG)

if(HAS_F_LTO_FLAG)
  option(ENABLE_LTO "Enable link-time optimization" OFF)

endif(HAS_F_LTO_FLAG)

if(ENABLE_LTO)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif(ENABLE_LTO)

add_subdirectory(quickjs "${CMAKE_BINARY_DIR}/quickjs")

link_directories(BEFORE ${quickjs_BINARY_DIR})
# message("quickjs_BINARY_DIR: ${quickjs_BINARY_DIR}")

# if(HAVE_EFENCE) if(USE_EFENCE) set(ELECTRICFENCE_LIBRARY efence) endif(USE_EFENCE) endif(HAVE_EFENCE)

link_directories("${CMAKE_BINARY_DIR}/quickjs")

function(src_program NAME SOURCES)
  foreach(SRC ${SOURCES})
    file(GLOB "${NAME}_SOURCES" ${SRC})
    list(APPEND SRCS ${${NAME}_SOURCES})
  endforeach()
  string(REGEX REPLACE ";" " " "SOURCES" "${SRCS}")
  string(REGEX REPLACE "/[^ ]*/" "\n  src/" "SOURCES" "${SOURCES}")
  set("${NAME}_SOURCES" ${SRCS})

  message("${NAME}_SOURCES: ${SOURCES}")
  add_executable(${NAME} src/${NAME}.cpp ${${NAME}_SOURCES})
  target_link_libraries(${NAME} ${ARGN})
  target_compile_definitions(${NAME} PRIVATE ${PLOTCV_DEFS})
  install(TARGETS ${NAME} DESTINATION bin)
endfunction()

option(HIGHGUI_VIEWER "Enable OpenCV highgui viewer" ON)
option(IMGUI_VIEWER "Enable imgui viewer" OFF)

if(HIGHGUI_VIEWER)
  include(highgui-viewer.cmake)
endif(HIGHGUI_VIEWER)

if(IMGUI_VIEWER)
  include(imgui-viewer.cmake)
endif(IMGUI_VIEWER)

set(QUICKJS_SOURCES_LIST "quickjs/quickjs.c;quickjs/quickjs.hpp;quickjs/libregexp.c;quickjs/libunicode.c;quickjs/cutils.c;quickjs/quickjs-libc.c;quickjs/quickjs-libc.h;quickjs/quickjs-find-module.c;quickjs/libbf.c")
# add_definitions(-D_GNU_SOURCE=1)

macro(find_static_library VAR NAME)
  find_library(${VAR} NAMES lib${NAME}.a PATHS "${CMAKE_INSTALL_PREFIX}/lib" NO_DEFAULT_PATH)
endmacro()

set(QUICKJS_LIBRARY quickjs)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/quickjs)

message("QUICKJS_LIBRARY: ${QUICKJS_LIBRARY}")

src_program(
  cli "src/cli.cpp;src/plot-cv.cpp;src/color.cpp;src/matrix.cpp;src/line.cpp;src/geometry.cpp;src/util.cpp;src/js.cpp;src/jsbindings.cpp;src/js_contour.cpp;src/js.cpp;src/js_draw.cpp;src/js_line.cpp;src/js_mat.cpp;src/js_point.cpp;src/js_point_iterator.cpp;src/js_rect.cpp;src/js_size.cpp;src/js_cv.cpp;src/js_video_capture.cpp" ${OpenCV_LIBS} ${QUICKJS_LIBRARY} pthread dl m ${LIBWS2_32}
  #
)

target_compile_definitions(cli PRIVATE _GNU_SOURCE=1 CONFIG_VERSION="${quickjs_version}" CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM CONFIG_CHECK_JSVALUE ${PLOTCV_DEFS})

add_executable(test-qjs test-qjs.c)

target_link_libraries(test-qjs quickjs pthread dl m)

target_compile_definitions(test-qjs PRIVATE _GNU_SOURCE=1 CONFIG_VERSION="${quickjs_version}" CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM=1 ${PLOTCV_DEFS})

option(ADDITIONAL_PROGS "Build other OpenCV programs" FALSE)

# option(QUICKJS_MODULES "Build .so modules for quickjs" "${BUILD_SHARED_LIBS}")
option(QUICKJS_MODULES "Build .so modules for quickjs" ON)

if(QUICKJS_MODULES)
  include(js-bindings.cmake)
endif(QUICKJS_MODULES)
#[[
add_executable(test-dlfcn test-dlfcn.c)
target_link_libraries(test-dlfcn dl)
]]

# if(ADDITIONAL_PROGS)
add_subdirectory(opencv ${CMAKE_CURRENT_BINARY_DIR}/opencv)
# endif(ADDITIONAL_PROGS)
