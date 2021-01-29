include_directories(${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/opencv)

macro(add_prog)
  set(ARGS ${ARGN})
  list(GET ARGS 0 name)
  list(REMOVE_AT ARGS 0)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cpp ${ARGS})
  target_link_libraries(${name} ${OpenCV_LIBS})
  target_include_directories(${name} PUBLIC ${OPENCV_INCLUDE_DIRS})
  # install(TARGETS ${name} DESTINATION bin)
endmacro()

unset(EXTRA_PROGS)
set(EXTRA_PROGS "" CACHE STRING "Compiled extra programs" FORCE)

if(ADDITIONAL_PROGS)
  # add_prog(mouth) add_prog(contour) add_prog(contours2) add_prog(distance)
  add_prog(images ${CMAKE_SOURCE_DIR}/src/polygon.cpp ${CMAKE_SOURCE_DIR}/src/data.cpp)
  # add_prog(laser) add_prog(o_angle) add_prog(code7-3) add_prog(connected_components) add_prog(generalContours_demo1) add_prog(generalContours_demo2) add_prog(hull_demo) add_prog(imageSegmentation) add_prog(imgproc_drawContours) add_prog(pointPolygonTest_demo) add_prog(watershed) add_prog(bg_sub) add_prog(bgsubtract2) add_prog(calibration) add_prog(camera_image) add_prog(camera_opencv)
  # add_prog(camshift) add_prog(cascade_detect_camera) add_prog(clahe) add_prog(code10-1) add_prog(code4-3) add_prog(code4-4) add_prog(code4-5) add_prog(code5-7) add_prog(code7-2) add_prog(code9-2) add_prog(computeSaliency) add_prog(dbt_face_detection) add_prog(detect) add_prog(example) add_prog(fback) add_prog(image_sequence) add_prog(live_demo) add_prog(laplace) add_prog(findLaserPoint)
  # add_prog(homography) add_prog(meanshift) add_prog(motempl) add_prog(OpenCVFaceDetection) add_prog(OpenCVWebcam) add_prog(peopledetect) add_prog(polar_transforms) add_prog(projectorcalibration) add_prog(seeds) add_prog(SimpleGrab) add_prog(slic) add_prog(starter_video) add_prog(targeting2) add_prog(template) add_prog(Threshold_inRange) add_prog(ufacedetect) add_prog(videocapture_basic)
  # add_prog(videocapture_gphoto2_autofocus) add_prog(videocapture_image_sequence) add_prog(videocapture_openni) add_prog(videocapture_pvapi) add_prog(videocapture_starter) add_prog(video-input-psnr-ssim) add_prog(video-write) add_prog(videowriter_basic) add_prog(warpPerspective_demo) add_prog(decompose_homography) add_prog(perspective_correction) add_prog(lab-contrast) add_prog(homomorphicFilter)
  # add_prog(1a.Test) add_prog(2d.Capture) add_prog(4a.Histogram) add_prog(4b.Equalizing) add_prog(4c.Threshold) add_prog(5.Motion_detect) add_prog(6a.Select) add_prog(6c.Select_cam) add_prog(7.Track) add_prog(9a.Training) add_prog(9b.Object_detect) add_prog(bgfg) add_prog(calibrate_camera_charuco) add_prog(calibrate_camera) add_prog(cap_pattern) add_prog(capsinpattern) add_prog(code5-8)
  # add_prog(dense_optical_flow) add_prog(detect_diamonds) add_prog(dis_opticalflow) add_prog(facemark_lbf_fitting) add_prog(filterdemo) add_prog(mace_webcam) add_prog(main ${CMAKE_CURRENT_SOURCE_DIR}/getExteriorContour.cpp ${CMAKE_CURRENT_SOURCE_DIR}/detectLines.cpp) add_prog(oil) add_prog(optical_flow_dense) add_prog(qrcode) add_prog(videocapture_gstreamer_pipeline)
  # add_prog(homography_from_camera_displacement) add_prog(aruco_ar_demo) add_prog(bounding_rect_circle) add_prog(detect_checked_boxes) add_prog(detect_rectangles) add_prog(morpho) add_prog(test2)

  # add_prog(segment_objects) add_prog(tracker) add_prog(tutorial_multitracker)

  # add_prog(objectDetection) add_prog(objectDetection2) add_prog(obj_track)
  set(EXTRA_PROGS
      ${EXTRA_PROGS}
      mouth
      contour
      contours2
      distance
      laser
      o_angle
      code7-3
      connected_components
      generalContours_demo1
      generalContours_demo2
      hull_demo
      imageSegmentation
      imgproc_drawContours
      pointPolygonTest_demo
      watershed
      bg_sub
      bgsubtract2
      calibration
      camera_image
      camera_opencv
      camshift
      cascade_detect_camera
      clahe
      code10-1
      code4-3
      code4-4
      code4-5
      code5-7
      code7-2
      code9-2
      computeSaliency
      dbt_face_detection
      detect
      example
      fback
      image_sequence
      live_demo
      laplace
      findLaserPoint
      homography
      meanshift
      motempl
      OpenCVFaceDetection
      OpenCVWebcam
      peopledetect
      polar_transforms
      projectorcalibration
      seeds
      SimpleGrab
      slic
      starter_video
      targeting2
      template
      Threshold_inRange
      ufacedetect
      videocapture_basic
      videocapture_gphoto2_autofocus
      videocapture_image_sequence
      videocapture_openni
      videocapture_pvapi
      videocapture_starter
      video-input-psnr-ssim
      video-write
      videowriter_basic
      warpPerspective_demo
      decompose_homography
      perspective_correction
      lab-contrast
      homomorphicFilter
      1a.Test
      2d.Capture
      4a.Histogram
      4b.Equalizing
      4c.Threshold
      5.Motion_detect
      6a.Select
      6c.Select_cam
      7.Track
      9a.Training
      9b.Object_detect
      bgfg
      calibrate_camera_charuco
      calibrate_camera
      cap_pattern
      capsinpattern
      code5-8
      dense_optical_flow
      detect_diamonds
      dis_opticalflow
      facemark_lbf_fitting
      filterdemo
      mace_webcam
      oil
      optical_flow_dense
      qrcode
      videocapture_gstreamer_pipeline
      homography_from_camera_displacement
      aruco_ar_demo
      bounding_rect_circle
      detect_checked_boxes
      detect_rectangles
      morpho
      test2
      segment_objects
      tracker
      tutorial_multitracker
      objectDetection
      objectDetection2
      obj_track)

endif(ADDITIONAL_PROGS)
if(ADDITIONAL_PROGS_2)

  set(EXTRA_PROGS
      ${EXTRA_PROGS}
      8.Face_eye_detect
      a
      adaptiveThresholding
      alexnet
      api_example
      App
      aruco_simple
      backend_plugin
      benchmark
      bgfg_gmg
      bgfg_segm
      BGMain_Raspberry
      caffenet
      camera_calibration
      CamPaho
      camshiftdemo
      # cap cap_android_mediandk cap_aravis cap_dc1394_v2 cap_dshow cap_ffmpeg cap_gphoto2 cap_gstreamer cap_images cap_intelperc cap_librealsense cap_mfx_reader cap_mjpeg_decoder cap_msmf cap_pvapi capture_positives cap_v4l cap_ximea cap_xine
      cascadeclassifier
      cascadeclassifier_nvidia_api
      classification
      Code
      code10-3
      code6-4
      code6-5
      code7-1
      code7-5
      code8-1
      code8-2
      code8-3
      code8-4
      code8-5
      code9-3
      code9-5
      code9-6
      code9-7
      contours3
      contours4
      counter
      count_things
      csrt
      CV
      cvcap
      cvv_demo
      # d3d10_interop d3d11_interop d3d9ex_interop d3d9_interop
      Demo
      detect_board
      detect_board_charuco
      detect_markers
      distance_Measurement
      eyebrow
      facedetect
      face-recognition-start
      facerec_video
      fitellipse
      glsl-test
      gms_matcher
      googlenet
      gravador_source
      hog
      hybridtrackingsample
      image
      img_proc_7
      intelperc_capture
      introduction_to_pca
      ippasync_sample
      kcf
      kinfu_demo
      linemod
      lock
      magica
      main_detection
      main_pi_sdl_test
      main_ui
      mjpeg-opencv
      motionTracking
      motionTracking_modified
      mulobject
      multitracker
      MusicJamboree
      native
      netcvc
      objDetWithLoc
      object_detection
      ObjectProcessor
      omxcv-test
      opencl-opencv-interop
      opencv_test
      opengl_interop
      openni_capture
      paper-fix
      pct_webcam
      phase_corr
      pibilight
      piSender_aruco
      planar_tracking
      # qt_opengl raspicam_cv_stream_2
      retinaDemo
      retina_tutorial
      sampleDetectLandmarksvideo
      segmentation
      select3dobj
      shape_example
      SimplerBlobDetector
      smiledetect
      Source
      Source_teste
      squares
      stereo_multi
      targeting
      testlanedetector
      testlanedetector2
      text_detection
      tracking_by_matching
      train_HOG
      tutorial_customizing_cn_tracker
      tutorial_introduction_to_tracker
      tvl1_optical_flow
      usb_alexnet
      using_blobs
      videocapture_intelperc
      video_dmtx
      video_reader
      video_writer
      webcam_demo
      wrappers_video)
endif(ADDITIONAL_PROGS_2)

include(CheckIncludeFileCXX)

if(OPENCV_INCLUDE_DIRS AND EXISTS "${OPENCV_INCLUDE_DIRS}")
  list(APPEND CMAKE_REQUIRED_INCLUDES "${OPENCV_INCLUDE_DIRS}")
  include_directories("${OPENCV_INCLUDE_DIRS}")
endif(OPENCV_INCLUDE_DIRS AND EXISTS "${OPENCV_INCLUDE_DIRS}")

check_include_file_cxx(opencv2/xfeatures2d.hpp HAVE_XFEATURES2D)
message(STATUS "Have XFEATURES2D extra module: ${HAVE_XFEATURES2D}")
if(HAVE_XFEATURES2D)
  set(EXTRA_PROGS ${EXTRA_PROGS} rtObjectRecognition rtTracker video_homography)
endif(HAVE_XFEATURES2D)

if(ADDITIONAL_PROGS)
  find_package(Boost 1.56 REQUIRED COMPONENTS system)
  set(EXTRA_PROGS
      ${EXTRA_PROGS}
      Image-Recognition
      Measure-objects
      # rtTracker
      shapefinder
      Shape-Finder
      # Shape_Detect
      shape_detection
      voronoi)

  set(EXTRA_PROGS ${EXTRA_PROGS} cornerSubPix_Demo goodFeaturesToTrack_Demo lkdemo optical_flow pyrlk_optical_flow)

  set(EXTRA_PROGS
      ${EXTRA_PROGS} application_trace autofocus basicscanner canny CannyDetector_Demo circles code6-1 code6-2 contours Contour_detection detect_circle detect_geo_shapes detect_hough_circle documentScanner edge findContours_demo
      # getExteriorContour
      moments_demo Open-Source-CamScanner PhotoScanner readGraph scanner scannerLite videocapture_camera)

  add_prog(main detectLines.cpp getExteriorContour.cpp)
  target_link_libraries(Shape-Finder pthread)

  add_prog(Multiple_Object_Tracking_by_Image_Subtraction ${CMAKE_CURRENT_SOURCE_DIR}/Blob.cpp)
  target_link_libraries(Shape-Finder Boost::system)
endif(ADDITIONAL_PROGS)

if(EXTRA_PROGS)
  foreach(P ${EXTRA_PROGS})
    # add_prog("${P}")
  endforeach()
endif(EXTRA_PROGS)
