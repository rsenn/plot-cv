cmake_minimum_required(VERSION 3.0)
project(plot-cv)

set(DISABLE_WERROR TRUE CACHE BOOL "Disable -Werror flag" FORCE)                                                           

SET(CMAKE_BUILD_TYPE "Debug" CACHE STRING "build type")

SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo) 

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

macro(check_size TYPE VAR)
  check_type_size("${TYPE}" CMAKE_${VAR})
  if(${CMAKE_${VAR}})
    add_definitions(-D${VAR}=${CMAKE_${VAR}})
  endif(${CMAKE_${VAR}})
  message(STATUS "size of ${TYPE} ${CMAKE_${VAR}}")
endmacro(
  check_size
  TYPE
  VAR)

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
  message("Found math library: ${LIBM}")
endif(HAVE_LIBM)

check_library_exists(dl dlopen /usr/lib HAVE_LIBDL)
if(HAVE_LIBDL)
  set(LIBDL dl)
  message("Found dlfcn library: ${LIBDL}")
endif(HAVE_LIBDL)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_BINARY_DIR}")

check_library_exists(json-cpp ZN4YAML6detail9node_data9reset_mapEv /usr JSON_CPP)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE CACHE BOOL "Build installable binaries with -Wl,-rpath=PREFIX" FORCE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE CACHE BOOL "Add dirs in runtime search path to the install rpath" FORCE)

include(${CMAKE_CURRENT_SOURCE_DIR}/opencv.cmake)

include_directories(${OPENCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/opencv ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/quickjs)

check_cxx_compiler_flag("-std=c++20" CXX_STD_CXX_17_FLAG)
if(CXX_STD_CXX_17_FLAG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
endif(CXX_STD_CXX_17_FLAG)
check_cxx_compiler_flag("-fPIC" F_OPT_PIC)
if(F_OPT_PIC)
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fPIC")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC")
endif(F_OPT_PIC)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # option(USE_EFENCE "Enable electric fence" OFF)
  set(DEBUG_OUTPUT TRUE CACHE BOOL "Debug output" FORCE)
  add_definitions(-DDEBUG=1)


  check_cxx_compiler_flag("-O0" O_OPT_NONE)
  if(O_OPT_NONE)
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  endif(O_OPT_NONE)

endif(CMAKE_BUILD_TYPE STREQUAL "Debug")
if(DEBUG_OUTPUT)
  add_definitions(-DDEBUG_OUTPUT=1)
endif(DEBUG_OUTPUT)

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
#message("quickjs_BINARY_DIR: ${quickjs_BINARY_DIR}")

# if(HAVE_EFENCE) if(USE_EFENCE) set(ELECTRICFENCE_LIBRARY efence) endif(USE_EFENCE) endif(HAVE_EFENCE)

macro(add_prog)
  set(ARGS ${ARGN})
  list(GET ARGS 0 name)
  list(REMOVE_AT ARGS 0)
  add_executable(${name} opencv/${name}.cpp ${ARGS})
  target_link_libraries(${name} ${OpenCV_LIBS})
  install(TARGETS ${name} DESTINATION bin)
endmacro()

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


set(QUICKJS_SOURCES_LIST "quickjs/quickjs.c;quickjs/quickjs.h;quickjs/libregexp.c;quickjs/libunicode.c;quickjs/cutils.c;quickjs/quickjs-libc.c;quickjs/quickjs-libc.h;quickjs/quickjs-find-module.c;quickjs/libbf.c")
# add_definitions(-D_GNU_SOURCE=1)

macro(find_static_library VAR NAME)
  find_library(
    ${VAR}
    NAMES lib${NAME}.a
    PATHS "${CMAKE_INSTALL_PREFIX}/lib"
    NO_DEFAULT_PATH)
endmacro()

set(QUICKJS_LIBRARY quickjs)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/quickjs)

message("QUICKJS_LIBRARY: ${QUICKJS_LIBRARY}")

src_program(
  cli
  "src/cli.cpp;src/plot-cv.cpp;src/color.cpp;src/matrix.cpp;src/line.cpp;src/geometry.cpp;src/js.cpp;src/jsbindings.cpp;src/js_contour.cpp;src/js.cpp;src/js_draw.cpp;src/js_line.cpp;src/js_mat.cpp;src/js_point.cpp;src/js_point_iterator.cpp;src/js_rect.cpp;src/js_size.cpp;src/js_cv.cpp;src/js_video_capture.cpp"
  ${OpenCV_LIBS}
  ${QUICKJS_LIBRARY}
  pthread
  dl
  m
  ${LIBWS2_32}
  #
)

target_compile_definitions(cli PRIVATE _GNU_SOURCE=1 CONFIG_VERSION="${quickjs_version}" CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM CONFIG_CHECK_JSVALUE)

add_executable(test-qjs test-qjs.c)

target_link_libraries(test-qjs quickjs pthread dl m)

target_compile_definitions(test-qjs PRIVATE _GNU_SOURCE=1 CONFIG_VERSION="${quickjs_version}" CONFIG_PREFIX="${CMAKE_INSTALL_PREFIX}" CONFIG_BIGNUM=1)

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

if(ADDITIONAL_PROGS)
  add_prog(mouth)
  add_prog(canny)
  add_prog(contour)
  add_prog(Contour_detection)
  add_prog(contours2)
  # add_prog(contours4)
  add_prog(contours)
  add_prog(distance)
  add_prog(images src/polygon.cpp src/data.cpp)
  add_prog(laser)
  # 7u add_prog(MusicJamboree)
  add_prog(o_angle)
  add_prog(code6-1)
  add_prog(code6-2)
  add_prog(code7-3)
  add_prog(connected_components)
  add_prog(findContours_demo)
  add_prog(generalContours_demo1)
  add_prog(generalContours_demo2)
  add_prog(hull_demo)
  add_prog(imageSegmentation)
  add_prog(imgproc_drawContours)
  add_prog(moments_demo)
  add_prog(pointPolygonTest_demo)
  add_prog(watershed)
  add_prog(autofocus)
  add_prog(bg_sub)
  add_prog(bgsubtract2)
  add_prog(calibration)
  add_prog(camera_image)
  add_prog(camera_opencv)
  add_prog(camshift)
  add_prog(cascade_detect_camera)
  add_prog(circles)
  add_prog(clahe)
  add_prog(code10-1)
  add_prog(code4-3)
  add_prog(code4-4)
  add_prog(code4-5)
  add_prog(code5-7)
  add_prog(code7-2)
  add_prog(code9-2)
  add_prog(computeSaliency)
  add_prog(dbt_face_detection)
  add_prog(detect)
  add_prog(example)
  add_prog(fback)
  add_prog(image_sequence)
  add_prog(intelperc_capture)
  add_prog(live_demo)
  add_prog(laplace)
  add_prog(findLaserPoint)
  add_prog(homography)
  add_prog(meanshift)
  add_prog(motempl)
  add_prog(OpenCVFaceDetection)
  add_prog(OpenCVWebcam)
  add_prog(optical_flow)
  add_prog(peopledetect)
  add_prog(polar_transforms)
  add_prog(projectorcalibration)
  add_prog(pyrlk_optical_flow)
  add_prog(seeds)
  add_prog(SimpleGrab)
  add_prog(slic)
  add_prog(starter_video)
  add_prog(targeting2)
  add_prog(template)
  add_prog(Threshold_inRange)
  add_prog(ufacedetect)
  add_prog(videocapture_basic)
  add_prog(videocapture_camera)
  add_prog(videocapture_gphoto2_autofocus)
  add_prog(videocapture_image_sequence)
  add_prog(videocapture_intelperc)
  add_prog(videocapture_openni)
  # add_prog(VideoCapturePeopleCounter)
  add_prog(videocapture_pvapi)
  add_prog(videocapture_starter)
  add_prog(video-input-psnr-ssim)
  add_prog(video-write)
  add_prog(videowriter_basic)
  add_prog(warpPerspective_demo)
  add_prog(CannyDetector_Demo)
  add_prog(edge)
  add_prog(decompose_homography)
  add_prog(perspective_correction)
  add_prog(lab-contrast)
  add_prog(homomorphicFilter)
  add_prog(basicscanner)
  add_prog(1a.Test)
  add_prog(2d.Capture)
  add_prog(4a.Histogram)
  add_prog(4b.Equalizing)
  add_prog(4c.Threshold)
  add_prog(5.Motion_detect)
  add_prog(6a.Select)
  add_prog(6c.Select_cam)
  add_prog(7.Track)
  add_prog(9a.Training)
  add_prog(9b.Object_detect)
  # add_prog(api_example)
  add_prog(application_trace)
  add_prog(bgfg)
  add_prog(calibrate_camera_charuco)
  add_prog(calibrate_camera)
  add_prog(cap_pattern)
  add_prog(capsinpattern)
  add_prog(code5-8)
  add_prog(dense_optical_flow)
  # add_prog(detect_board_charuco) add_prog(detect_board)
  add_prog(detect_diamonds)
  add_prog(dis_opticalflow)
  add_prog(facemark_lbf_fitting)
  add_prog(filterdemo)
  # add_prog(gms_matcher) add_prog(kinfu_demo)
  add_prog(mace_webcam)
  add_prog(main opencv/detectLines.cpp opencv/getExteriorContour.cpp)
  add_prog(oil)
  add_prog(optical_flow_dense)
  # add_prog(pct_webcam)
  add_prog(qrcode)
  # add_prog(sampleDetectLandmarksvideo) add_prog(text_detection) add_prog(tracking_by_matching) add_prog(tutorial_introduction_to_tracker)
  add_prog(videocapture_gstreamer_pipeline)
  # add_prog(webcam_demo)
  add_prog(homography_from_camera_displacement)
  add_prog(aruco_ar_demo)
  # add_prog(tracking_calibration_with_img) target_link_libraries(tracking_calibration_with_img yaml-cpp roslib rospack roscpp )
endif(ADDITIONAL_PROGS)
