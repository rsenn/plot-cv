if(NOT OPENCV_CHECKED)
  message(CHECK_START "Finding opencv library")
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
            list(GET OPENCV_LIBRARY_DIRS 0 OpenCV_LIB_DIR)
set(OpenCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIRS})
    endif(NOT OpenCV_LIB_DIR)

        set(OPENCV_RESULT "")
              list(GET OPENCV_LIBRARY_DIRS 0 OPENCV_LIBRARY_DIR)

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

    message(CHECK_PASS "${OPENCV_RESULT}")
  else(OPENCV_FOUND OR OpenCV_LIBS)
    message(CHECK_FAIL "fail")
  endif(OPENCV_FOUND OR OpenCV_LIBS)
  set(OPENCV_CHECKED TRUE)
endif(NOT OPENCV_CHECKED)

