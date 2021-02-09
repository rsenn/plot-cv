macro(unset_all)
  foreach(VAR ${ARGN})
    unset("${VAR}" PARENT_SCOPE)
    unset("${VAR}" CACHE)
  endforeach(VAR ${ARGN})
endmacro(unset_all)

function(OPENCV_CHANGE VAR ACCESS VALUE LIST_FILE STACK)

  if("${VAR}" STREQUAL "OpenCV_Dir" OR "${VAR}" STREQUAL "OPENCV_ROOT")
    message("VAR ${VAR} changed!!!")
    unset(OPENCV_FOUND CACHE)
    unset(OPENCV_FOUND PARENT_SCOPE)
    unset(OPENCV_CHECKED CACHE)
    unset(OPENCV_CHECKED PARENT_SCOPE)

    unset_all(
      pkgcfg_lib_OPENCV_opencv_aruco
      pkgcfg_lib_OPENCV_opencv_bgsegm
      pkgcfg_lib_OPENCV_opencv_bioinspired
      pkgcfg_lib_OPENCV_opencv_calib3d
      pkgcfg_lib_OPENCV_opencv_ccalib
      pkgcfg_lib_OPENCV_opencv_core
      pkgcfg_lib_OPENCV_opencv_datasets
      pkgcfg_lib_OPENCV_opencv_dnn
      pkgcfg_lib_OPENCV_opencv_dnn_objdetect
      pkgcfg_lib_OPENCV_opencv_dnn_superres
      pkgcfg_lib_OPENCV_opencv_dpm
      pkgcfg_lib_OPENCV_opencv_face
      pkgcfg_lib_OPENCV_opencv_features2d
      pkgcfg_lib_OPENCV_opencv_flann
      pkgcfg_lib_OPENCV_opencv_freetype
      pkgcfg_lib_OPENCV_opencv_fuzzy
      pkgcfg_lib_OPENCV_opencv_hdf
      pkgcfg_lib_OPENCV_opencv_hfs
      pkgcfg_lib_OPENCV_opencv_highgui
      pkgcfg_lib_OPENCV_opencv_imgcodecs
      pkgcfg_lib_OPENCV_opencv_img_hash
      pkgcfg_lib_OPENCV_opencv_imgproc
      pkgcfg_lib_OPENCV_opencv_line_descriptor
      pkgcfg_lib_OPENCV_opencv_ml
      pkgcfg_lib_OPENCV_opencv_objdetect
      pkgcfg_lib_OPENCV_opencv_optflow
      pkgcfg_lib_OPENCV_opencv_phase_unwrapping
      pkgcfg_lib_OPENCV_opencv_photo
      pkgcfg_lib_OPENCV_opencv_plot
      pkgcfg_lib_OPENCV_opencv_quality
      pkgcfg_lib_OPENCV_opencv_reg
      pkgcfg_lib_OPENCV_opencv_rgbd
      pkgcfg_lib_OPENCV_opencv_saliency
      pkgcfg_lib_OPENCV_opencv_shape
      pkgcfg_lib_OPENCV_opencv_stereo
      pkgcfg_lib_OPENCV_opencv_stitching
      pkgcfg_lib_OPENCV_opencv_structured_light
      pkgcfg_lib_OPENCV_opencv_superres
      pkgcfg_lib_OPENCV_opencv_surface_matching
      pkgcfg_lib_OPENCV_opencv_text
      pkgcfg_lib_OPENCV_opencv_tracking
      pkgcfg_lib_OPENCV_opencv_video
      pkgcfg_lib_OPENCV_opencv_videoio
      pkgcfg_lib_OPENCV_opencv_videostab
      pkgcfg_lib_OPENCV_opencv_viz
      pkgcfg_lib_OPENCV_opencv_ximgproc
      pkgcfg_lib_OPENCV_opencv_xobjdetect
      pkgcfg_lib_OPENCV_opencv_xphoto)

  endif("${VAR}" STREQUAL "OpenCV_Dir" OR "${VAR}" STREQUAL "OPENCV_ROOT")
endfunction(
  OPENCV_CHANGE
  VAR
  ACCESS
  VALUE
  LIST_FILE
  STACK)
variable_watch(OpenCV_Dir OPENCV_CHANGE)

if(NOT OPENCV_CHECKED)
  message(STATUS "Finding opencv library")
  message("CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")
  message("CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

  if(OpenCV_Dir)
    string(REGEX REPLACE "/lib/.*" "" OPENCV_ROOT "${OpenCV_Dir}")
  endif(OpenCV_Dir)

  set(OPENCV_ROOT
      "${OPENCV_ROOT}"
      CACHE PATH "OpenCV root dir" FORCE)

  if(OPENCV_ROOT)
    list(APPEND CMAKE_PREFIX_PATH "${OPENCV_ROOT}")
    list(APPEND CMAKE_MODULE_PATH "${OPENCV_ROOT}/lib/cmake/opencv4")
  endif(OPENCV_ROOT)

  if(NOT OPENCV_FOUND)
    pkg_search_module(OPENCV REQUIRED NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH
                      opencv opencv4)

    if(OPENCV_FOUND)
      message("OpenCV with pkg_search_module")
    endif(OPENCV_FOUND)
  endif(NOT OPENCV_FOUND)

  if(NOT OPENCV_FOUND)
    find_package(OpenCV REQUIRED PATHS ${OPENCV_ROOT} NO_DEFAULT_PATH)

    if(OPENCV_FOUND)
      message("OpenCV with find_package")
    endif(OPENCV_FOUND)
  endif(NOT OPENCV_FOUND)

  if(OPENCV_FOUND OR OpenCV_LIBS)
    if(NOT OpenCV_LIBS)
      set(OpenCV_LIBS ${OPENCV_LIBRARIES})
    endif(NOT OpenCV_LIBS)
    if(NOT OpenCV_INCLUDE_DIRS)
      set(OpenCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIRS})
    endif(NOT OpenCV_INCLUDE_DIRS)
    if(NOT OpenCV_INCLUDE_DIR)
      if(OPENCV_INCLUDEDIR)
        set(OpenCV_INCLUDE_DIR "${OPENCV_INCLUDEDIR}")
      else(OPENCV_INCLUDEDIR)
        list(GET OPENCV_INCLUDE_DIRS 0 OpenCV_INCLUDE_DIR)
      endif(OPENCV_INCLUDEDIR)

    endif(NOT OpenCV_INCLUDE_DIR)
    if(NOT OpenCV_LIB_DIR)
      if(NOT OPENCV_LIBRARY_DIRS)
        set(OPENCV_LIBRARY_DIRS "")
      endif(NOT OPENCV_LIBRARY_DIRS)
      string(REGEX REPLACE ";.*" "" OpenCV_LIB_DIR "${OPENCV_LIBRARY_DIRS}")
      set(OpenCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIRS})
    endif(NOT OpenCV_LIB_DIR)

    set(OPENCV_RESULT "")
    set(OPENCV_LIBRARY_DIR "${OpenCV_LIB_DIR}")

    if(OPENCV_VERSION)
      set(OPENCV_NAME "${OPENCV_VERSION}")
      set(OPENCV_RESULT "version: ${OPENCV_VERSION}")
    endif(OPENCV_VERSION)

    if(OpenCV_LIB_DIR)
      if(NOT OPENCV_NAME)
        set(OPENCV_NAME "${OpenCV_LIB_DIR}")
      endif(NOT OPENCV_NAME)

      set(OPENCV_RESULT "${OPENCV_RESULT} directory: ${OpenCV_LIB_DIR}")
    endif(OpenCV_LIB_DIR)

    link_directories(${OpenCV_LIB_DIR})
    include_directories(${OpenCV_INCLUDE_DIRS})

    set(CMAKE_INSTALL_RPATH "${OpenCV_LIB_DIR}:${CMAKE_INSTALL_RPATH}")
    set(CMAKE_BUILD_RPATH "${OpenCV_LIB_DIR}:${CMAKE_BUILD_RPATH}")

    message(STATUS "${OPENCV_RESULT}")
  else(OPENCV_FOUND OR OpenCV_LIBS)
    message(STATUS "fail")
  endif(OPENCV_FOUND OR OpenCV_LIBS)

  if(OPENCV_FOUND)
    message("OpenCV found: ${OPENCV_PREFIX}")
    set(OPENCV_PREFIX
        "${OPENCV_PREFIX}"
        CACHE PATH "OpenCV install prefix" FORCE)
  endif(OPENCV_FOUND)
  set(OPENCV_CHECKED TRUE)
endif(NOT OPENCV_CHECKED)
