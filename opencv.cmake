

function(OPENCV_CHANGE VAR ACCESS VALUE LIST_FILE STACK)

  if("${VAR}" STREQUAL "OpenCV_Dir" OR "${VAR}" STREQUAL "OPENCV_ROOT")
  unset(OPENCV_FOUND CACHE)
  unset(OPENCV_FOUND)

  endif("${VAR}" STREQUAL "OpenCV_Dir" OR "${VAR}" STREQUAL "OPENCV_ROOT")
endfunction(OPENCV_CHANGE VAR ACCESS VALUE LIST_FILE STACK)
variable_watch(OpenCV_Dir OPENCV_CHANGE)

if(NOT OPENCV_CHECKED)
  message(STATUS "Finding opencv library")

if(OpenCV_Dir)
set(OPENCV_ROOT "${OpenCV_Dir}")
endif(OpenCV_Dir)

  if(OPENCV_ROOT)
    list(APPEND CMAKE_PREFIX_PATH "${OPENCV_ROOT}")
  endif(OPENCV_ROOT)

  if(NOT OPENCV_FOUND)
    pkg_search_module(OPENCV REQUIRED NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH opencv opencv4)
  endif(NOT OPENCV_FOUND)

  if(NOT OPENCV_FOUND)
    find_package(OpenCV REQUIRED)
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
  set(OPENCV_CHECKED TRUE)
endif(NOT OPENCV_CHECKED)
