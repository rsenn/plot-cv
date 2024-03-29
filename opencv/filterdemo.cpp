/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2017, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>

#include <stdio.h>

// using namespace cv;
using namespace std;

int
main(int argc, const char** argv) {
  float alpha = 1.0f;
  float sigma = 0.02f;
  int rows0 = 480;
  int niters = 10;
  cv::Mat frame, src, dst;

  const char* window_name = "Anisodiff : Exponential Flux";

  cv::VideoCapture cap;
  if(argc > 1)
    cap.open(argv[1]);
  else
    cap.open(0);

  if(!cap.isOpened()) {
    printf("Cannot initialize video capturing\n");
    return 0;
  }

  // Create a window
  cv::namedWindow(window_name, 1);

  // create a toolbar
  cv::createTrackbar("No. of time steps", window_name, &niters, 30, 0);

  for(;;) {
    cap >> frame;
    if(frame.empty())
      break;

    if(frame.rows <= rows0)
      src = frame;
    else
      cv::resize(frame, src, cv::Size(cvRound(480. * frame.cols / frame.rows), 480), 0, 0, cv::INTER_LINEAR_EXACT);

    float t = (float)cv::getTickCount();
    cv::ximgproc::anisotropicDiffusion(src, dst, alpha, sigma, niters);
    t = (float)cv::getTickCount() - t;
    printf("time: %.1fms\n", t * 1000. / cv::getTickFrequency());
    cv::imshow(window_name, dst);

    // Wait for a key stroke; the same function arranges events processing
    char c = (char)cv::waitKey(30);
    if(c >= 0)
      break;
  }

  return 0;
}
