project(plot-cv C CXX)

message("Building project [1;31m${PROJECT_NAME}[0m")

set(DISABLE_WERROR TRUE CACHE BOOL "Disable -Werror flag" FORCE)

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "build type")

if(CACHE{CMAKE_BUILD_TYPE})
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
endif(CACHE{CMAKE_BUILD_TYPE})

set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Build with local rpath" FORCE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Build with install rpath" FORCE)

set(QUICKJS_SOURCES_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/quickjs)

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
include(FindPNG)
if(NOT PKG_CONFIG_FOUND)
  include(FindPkgConfig)
endif(NOT PKG_CONFIG_FOUND)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MoreFunctions.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/OpenCV.cmake)
#include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPNG.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/quickjs/cmake/FindQuickJS.cmake)

option(BUILD_QUICKJS "Build quickjs library" ON)
option(BUILD_QJS_OPENCV "Build qjs-opencv module" ON)
if(PNG_LIBRARIES)
  message(STATUS "Found libpng library: ${PNG_LIBRARIES}")
endif(PNG_LIBRARIES)

if(BUILD_QUICKJS)
  include(${CMAKE_CURRENT_SOURCE_DIR}/quickjs/defs.cmake)
else(BUILD_QUICKJS)
  #dump(QUICKJS_PREFIX)

  find_quickjs()
  #dump(QUICKJS_INCLUDE_DIR)
endif(BUILD_QUICKJS)

find_opencv()

if(ENABLE_CCACHE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CCache.cmake)
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
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

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
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}")

check_library_exists(json-cpp ZN4YAML6detail9node_data9reset_mapEv /usr JSON_CPP)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE CACHE BOOL "Build installable binaries with -Wl,-rpath=PREFIX" FORCE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE CACHE BOOL "Add dirs in runtime search path to the install rpath" FORCE)

include_directories(
  ${OPENCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/opencv ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/quickjs)

if(NOT CXX_STANDARD_FLAG)
  foreach(CXX_STANDARD c++17 c++14 c++11)
    string(REPLACE "c++" "CPLUSPLUS" CXX_STANDARD_NUM "${CXX_STANDARD}")
    set(CMAKE_REQUIRED_QUIET TRUE)
    check_cxx_compiler_flag("-std=${CXX_STANDARD}" CXX_STANDARD_${CXX_STANDARD_NUM})
    set(CMAKE_REQUIRED_QUIET FALSE)
    if(CXX_STANDARD_${CXX_STANDARD_NUM})
      string(REGEX REPLACE "\\+" "x" CXX_STANDARD_NAME "${CXX_STANDARD_VALUE}")
      string(TOUPPER "${CXX_STANDARD_NAME}" CXX_STANDARD_NAME)
      string(REGEX REPLACE "CXX" "" CXX_STANDARD_VERSION "${CXX_STANDARD_NAME}")

      if(NOT CXX_STANDARD_NAME STREQUAL "")
        add_definitions(-D"${CXX_STANDARD_NAME}")
      endif(NOT CXX_STANDARD_NAME STREQUAL "")
      # add_definitions(-DCXX_STANDARD=${CXX_STANDARD_VERSION})

      set(CXX_STANDARD_VALUE "${CXX_STANDARD}" CACHE STRING "C++ standard" FORCE)
      set(CXX_STANDARD_FLAG "-std=${CXX_STANDARD}" CACHE STRING "C++ standard argument" FORCE)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_STANDARD_FLAG}")
      break()
    endif(CXX_STANDARD_${CXX_STANDARD_NUM})
  endforeach()
else(NOT CXX_STANDARD_FLAG)
  string(REPLACE "-std=" "" CXX_STANDARD_VALUE "${CXX_STANDARD_FLAG}")
endif(NOT CXX_STANDARD_FLAG)

message("C++ standard: ${CXX_STANDARD_VALUE}")

macro(NOWARN_FLAG FLAG)
  canonicalize(VARNAME "${FLAG}")
  check_c_compiler_flag("${FLAG}" "${VARNAME}")
  #dump(${VARNAME})

  if(${VARNAME})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAG}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")

  endif(${VARNAME})
endmacro(NOWARN_FLAG FLAG)

string(REGEX REPLACE " -Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
string(REGEX REPLACE " -Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

nowarn_flag(-Wno-unused-value)
nowarn_flag(-Wno-unused-variable)

set(CMAKE_CXX11_EXTENSION_COMPILE_OPTION "${CXX_STANDARD_FLAG}" CACHE STRING "" FORCE)

if("${CMAKE_CXX_COMPILER_ID}" MATCHES ".*Clang.*")
  nowarn_flag(-Wno-deprecated-anon-enum-enum-conversion)
  nowarn_flag(-Wno-extern-c-compat)
  nowarn_flag(-Wno-implicit-int-float-conversion)
  nowarn_flag(-Wno-deprecated-enum-enum-conversion)
endif("${CMAKE_CXX_COMPILER_ID}" MATCHES ".*Clang.*")

#dump(CMAKE_C_FLAGS CMAKE_CXX_FLAGS)
#dump(CMAKE_CXX_FLAGS_DEBUG)

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

# if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")

# option(USE_EFENCE "Enable electric fence" OFF)
#set(DEBUG_OUTPUT TRUE CACHE BOOL "Debug output" FORCE)
#set(DEBUG_MAT FALSE CACHE BOOL "Debug cv::Mat" FORCE)
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

# endif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")

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

link_directories(BEFORE ${quickjs_BINARY_DIR})
# message("quickjs_BINARY_DIR: ${quickjs_BINARY_DIR}")

# if(HAVE_EFENCE) if(USE_EFENCE) set(ELECTRICFENCE_LIBRARY efence)
# endif(USE_EFENCE) endif(HAVE_EFENCE)

link_directories("${CMAKE_CURRENT_BINARY_DIR}/quickjs" "${QUICKJS_LIBRARY_DIR}")

function(src_program NAME SOURCES)
  foreach(SRC ${SOURCES})
    file(GLOB "${NAME}_SOURCES" ${SRC})
    list(APPEND SRCS ${${NAME}_SOURCES})
  endforeach()
  string(REGEX REPLACE ";" " " "SOURCES" "${SRCS}")
  string(REGEX REPLACE "/[^ ]*/" "\n  src/" "SOURCES" "${SOURCES}")
  set("${NAME}_SOURCES" ${SRCS})

  # message("${NAME}_SOURCES: ${SOURCES}")
  add_executable(${NAME} src/${NAME}.cpp ${${NAME}_SOURCES})
  target_link_libraries(${NAME} ${ARGN})
  target_compile_definitions(${NAME} PRIVATE ${PLOTCV_DEFS})
  install(TARGETS ${NAME} DESTINATION bin)
endfunction()

option(HIGHGUI_VIEWER "Enable OpenCV highgui viewer" OFF)
option(IMGUI_VIEWER "Enable imgui viewer" OFF)

if(HIGHGUI_VIEWER)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/highgui-viewer.cmake)
endif(HIGHGUI_VIEWER)

if(IMGUI_VIEWER)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/imgui-viewer.cmake)
endif(IMGUI_VIEWER)

set(QUICKJS_SOURCES_LIST
    "quickjs/quickjs.c;quickjs/quickjs.hpp;quickjs/libregexp.c;quickjs/libunicode.c;quickjs/cutils.c;quickjs/quickjs-libc.c;quickjs/quickjs-libc.h;quickjs/quickjs-find-module.c;quickjs/libbf.c"
)
# add_definitions(-D_GNU_SOURCE=1)

macro(find_static_library VAR NAME)
  find_library(${VAR} NAMES lib${NAME}.a PATHS "${CMAKE_INSTALL_PREFIX}/lib" NO_DEFAULT_PATH)
endmacro()

set(QUICKJS_LIBRARY quickjs)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/quickjs)

list(
  APPEND
  CLI_SOURCES
  #qjs-opencv/geometry.cpp
  #qjs-opencv/jsbindings.cpp
  #qjs-opencv/js_contour.cpp
  #qjs-opencv/js_cv.cpp
  #qjs-opencv/js_draw.cpp
  #qjs-opencv/js_keypoint.cpp
  #qjs-opencv/js_line.cpp
  #qjs-opencv/js_mat.cpp
  #qjs-opencv/js_point.cpp
  #qjs-opencv/js_point_iterator.cpp
  #qjs-opencv/js_rect.cpp
  #qjs-opencv/js_rotated_rect.cpp
  #qjs-opencv/js_size.cpp
  #qjs-opencv/js_umat.cpp
  #qjs-opencv/js_video_capture.cpp
  #qjs-opencv/line.cpp
  #qjs-opencv/util.cpp
  src/cli.cpp
  src/color.cpp
  src/js.cpp
  src/matrix.cpp
  src/plot-cv.cpp)

#dump(PNG_LIBRARY PNG_LIBRARY_RELEASE)

option(BUILD_CLI "Build command-line interpreter" OFF)

#include(CheckLibraryExists)
#check_library_exists(opencv_freetype _ZN2cv8freetype15createFreeType2Ev "${OPENCV_LIBRARY_DIRS}" HAVE_OPENCV2_FREETYPE_LIBRARY)
#
#if(HAVE_OPENCV2_FREETYPE_LIBRARY)
#  set(OPENCV_FREETYPE_LIBRARY opencv_freetype)
#endif(HAVE_OPENCV2_FREETYPE_LIBRARY)
#
#dump(OPENCV_FREETYPE_LIBRARY)

include(CheckIncludeFileCXX)

set(CMAKE_REQUIRED_INCLUDES "${OPENCV_INCLUDE_DIRS}")
check_include_file_cxx(opencv2/freetype.hpp HAVE_OPENCV2_FREETYPE_HPP)
if(HAVE_OPENCV2_FREETYPE_HPP)
  #add_definitions(-DHAVE_OPENCV_FREETYPE)
  set(OPENCV_FREETYPE_LIBRARY opencv_freetype)
endif(HAVE_OPENCV2_FREETYPE_HPP)
set(OPENCV_FREETYPE_LIBRARY opencv_freetype)

if(BUILD_CLI)
  src_program(
    cli
    "${CLI_SOURCES}"
    ${OpenCV_LIBS}
    ${QUICKJS_LIBRARY}
    ${PNG_LIBRARY}
    #png
    pthread
    dl
    m
    ${LIBWS2_32}
    #
  )

  target_compile_definitions(
    cli
    PRIVATE _GNU_SOURCE=1
            #CONFIG_VERSION="${QUICKJS_VERSION}"
            CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM CONFIG_CHECK_JSVALUE ${PLOTCV_DEFS})
  target_link_libraries(cli ${QUICKJS_OPENCV_A})
  target_link_libraries(cli ${PNG_LIBRARIES} ${OPENCV_FREETYPE_LIBRARY})

  add_dependencies(cli qjs-opencv)
endif(BUILD_CLI)

#add_executable(test-qjs test-qjs.c)
#target_link_libraries(test-qjs quickjs pthread dl m)
#target_compile_definitions(test-qjs PRIVATE _GNU_SOURCE=1 CONFIG_VERSION="${QUICKJS_VERSION}"
#                                            CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM=1 ${PLOTCV_DEFS})

option(ADDITIONAL_PROGS "Build other OpenCV programs" OFF)

# option(QUICKJS_MODULES "Build .so modules for quickjs" "${BUILD_SHARED_LIBS}")
option(QUICKJS_MODULES "Build .so modules for quickjs" ON)

if(QUICKJS_MODULES)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/JSBindings.cmake)
endif(QUICKJS_MODULES)
#[[
add_executable(test-dlfcn test-dlfcn.c)
target_link_libraries(test-dlfcn dl)
]]

if(BUILD_QJS_OPENCV)
  add_subdirectory(qjs-opencv "${CMAKE_CURRENT_BINARY_DIR}/qjs-opencv")
endif(BUILD_QJS_OPENCV)

option(BUILD_QUICKJS "Build QuickJS" ON)
if(BUILD_QUICKJS)
  add_subdirectory(quickjs "${CMAKE_CURRENT_BINARY_DIR}/quickjs")
endif(BUILD_QUICKJS)

if(ADDITIONAL_PROGS)
  add_subdirectory(opencv ${CMAKE_CURRENT_BINARY_DIR}/opencv)
endif(ADDITIONAL_PROGS)

string(REPLACE "/build/" "/inst/" CMAKE_INSTALL_DIR "${CMAKE_BINARY_DIR}")
string(TIMESTAMP DATE_TIME "%y%m%d" UTC)

add_custom_target(
  dist
  COMMAND rm -rf "${CMAKE_INSTALL_DIR}"
  COMMAND ${CMAKE_MAKE_PROGRAM} install DESTDIR="${CMAKE_INSTALL_DIR}"
  COMMAND tar -C "${CMAKE_INSTALL_DIR}" -J -cf "${CMAKE_PROJECT_NAME}-${DATE_TIME}.tar.xz" .
  #COMMAND 7z a -w"${CMAKE_INSTALL_DIR}"  "${CMAKE_PROJECT_NAME}-${DATE_TIME}.tar"  .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  BYPRODUCTS "${CMAKE_INSTALL_DIR}"
  COMMENT "Install into directory")

install(FILES meriyah.js eagle2gerber.js xml2react.js convert-imports.js create-tamper.js cv-shell.js eagle2svg.js
              eagle-shell.js extract-structs.js takephoto.js xml2js.js DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
