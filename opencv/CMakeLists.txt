include_directories(${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/opencv)

macro(add_prog)
  set(ARGS ${ARGN})
  list(GET ARGS 0 name)
  list(REMOVE_AT ARGS 0)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cpp ${ARGS})
  target_link_libraries(${name} ${OpenCV_LIBS})
  install(TARGETS ${name} DESTINATION bin)
endmacro()

add_prog(mouth)
add_prog(canny)
add_prog(contour)
add_prog(Contour_detection)
add_prog(contours2)
# add_prog(contours4)
add_prog(contours)
add_prog(distance)
add_prog(images ${CMAKE_SOURCE_DIR}/src/polygon.cpp ${CMAKE_SOURCE_DIR}/src/data.cpp)
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
# add_prog(intelperc_capture)
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
# add_prog(videocapture_intelperc)
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
add_prog(main ${CMAKE_CURRENT_SOURCE_DIR}/detectLines.cpp ${CMAKE_CURRENT_SOURCE_DIR}/getExteriorContour.cpp)
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
#[[
add_prog(a)
add_prog(aruco_simple)
add_prog(benchmark)
add_prog(bgfg_gmg)
add_prog(BGMain_Raspberry)
add_prog(camera_calibration)
add_prog(camshiftdemo)
add_prog(cap_android_mediandk)
add_prog(cap_aravis)
add_prog(cap_dshow)
add_prog(cap_gphoto2)
add_prog(cap_gstreamer)
add_prog(cap_mjpeg_decoder)
add_prog(cap_msmf)
add_prog(cap_v4l)
add_prog(cap_ximea)
add_prog(cascadeclassifier)
add_prog(cascadeclassifier_nvidia_api)
add_prog(code7-1)
add_prog(code7-5)
add_prog(code8-1)
add_prog(code8-2)
add_prog(code8-3)
add_prog(code8-4)
add_prog(code8-5)
add_prog(code9-5)
add_prog(code9-6)
add_prog(code9-7)
add_prog(code10-3)
add_prog(count_things)
add_prog(csrt)
add_prog(CV)
add_prog(d3d9ex_interop)
add_prog(d3d9_interop)
add_prog(Demo)
add_prog(facedetect)
add_prog(face-recognition-start)
add_prog(hybridtrackingsample)
add_prog(img_proc_7)
add_prog(kcf)
add_prog(linemod)
add_prog(lkdemo)
add_prog(Main)
add_prog(main_detection)
add_prog(motionTracking)
add_prog(motionTracking_modified)
add_prog(mulobject)
add_prog(multitracker)
add_prog(objectDetection)
add_prog(objectDetection2)
add_prog(ObjectProcessor)
add_prog(object_detection)
add_prog(obj_track)
add_prog(opencl-opencv-interop)
add_prog(opencv_test)
add_prog(opengl_interop)
add_prog(openni_capture)
add_prog(planar_tracking)
add_prog(qt_opengl)
add_prog(segment_objects)
add_prog(select3dobj)
add_prog(smiledetect)
add_prog(stereo_multi)
add_prog(targeting)
add_prog(tracker)
add_prog(train_HOG)
add_prog(tutorial_customizing_cn_tracker)
add_prog(tutorial_multitracker)
add_prog(using_blobs)
add_prog(video)
add_prog(video_homography)
]]