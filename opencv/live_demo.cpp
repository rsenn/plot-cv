/*
 *  By downloading, copying, installing or using the software you agree to this license.
 *  If you do not agree to this license, do not download, install,
 *  copy or use the software.
 *
 *
 *  License Agreement
 *  For Open Source Computer Vision Library
 *  (3 - clause BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met :
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and / or other materials provided with the distribution.
 *
 *  * Neither the names of the copyright holders nor the names of the contributors
 *  may be used to endorse or promote products derived from this software
 *  without specific prior written permission.
 *
 *  This software is provided by the copyright holders and contributors "as is" and
 *  any express or implied warranties, including, but not limited to, the implied
 *  warranties of merchantability and fitness for a particular purpose are disclaimed.
 *  In no event shall copyright holders or contributors be liable for any direct,
 *  indirect, incidental, special, exemplary, or consequential damages
 *  (including, but not limited to, procurement of substitute goods or services;
 *  loss of use, data, or profits; or business interruption) however caused
 *  and on any theory of liability, whether in contract, strict liability,
 *  or tort(including negligence or otherwise) arising in any way out of
 *  the use of this software, even if advised of the possibility of such damage.
 */

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
// using namespace cv;
using namespace cv::ximgproc;

#include <iostream>
using namespace std;

typedef void (*FilteringOperation)(const cv::Mat& src, cv::Mat& dst);
// current mode (filtering operation example)
FilteringOperation g_filterOp = NULL;

// list of filtering operations
void filterDoNothing(const cv::Mat& frame, cv::Mat& dst);
void filterBlurring(const cv::Mat& frame, cv::Mat& dst);
void filterStylize(const cv::Mat& frame, cv::Mat& dst);
void filterDetailEnhancement(const cv::Mat& frame8u, cv::Mat& dst);

// common sliders for every mode
int g_sigmaColor = 25;
int g_sigmaSpatial = 10;

// for Stylizing mode
int g_edgesGamma = 100;

// for Details Enhancement mode
int g_contrastBase = 100;
int g_detailsLevel = 100;

int g_numberOfCPUs = cv::getNumberOfCPUs();

// We will use two callbacks to change parameters
void changeModeCallback(int state, void* filter);
void changeNumberOfCpuCallback(int count, void*);

void splitScreen(const cv::Mat& rawFrame, cv::Mat& outputFrame, cv::Mat& srcFrame, cv::Mat& processedFrame);

// trivial filter
void
filterDoNothing(const cv::Mat& frame, cv::Mat& dst) {
  frame.copyTo(dst);
}

// simple edge-aware blurring
void
filterBlurring(const cv::Mat& frame, cv::Mat& dst) {
  cv::ximgproc::dtFilter(frame, frame, dst, g_sigmaSpatial, g_sigmaColor, DTF_RF);
}

// stylizing filter
void
filterStylize(const cv::Mat& frame, cv::Mat& dst) {
  // blur frame
  cv::Mat filtered;
  cv::ximgproc::dtFilter(frame, frame, filtered, g_sigmaSpatial, g_sigmaColor, DTF_NC);

  // compute grayscale blurred frame
  cv::Mat filteredGray;
  cv::cvtColor(filtered, filteredGray, cv::COLOR_BGR2GRAY);

  // find gradients of blurred image
  cv::Mat gradX, gradY;
  cv::Sobel(filteredGray, gradX, CV_32F, 1, 0, 3, 1.0 / 255);
  cv::Sobel(filteredGray, gradY, CV_32F, 0, 1, 3, 1.0 / 255);

  // compute magnitude of gradient and fit it accordingly the gamma parameter
  cv::Mat gradMagnitude;
  cv::magnitude(gradX, gradY, gradMagnitude);
  cv::pow(gradMagnitude, g_edgesGamma / 100.0, gradMagnitude);

  // multiply a blurred frame to the value inversely proportional to the magnitude
  cv::Mat multiplier = 1.0 / (1.0 + gradMagnitude);
  cv::cvtColor(multiplier, multiplier, cv::COLOR_GRAY2BGR);
  cv::multiply(filtered, multiplier, dst, 1, dst.type());
}

void
filterDetailEnhancement(const cv::Mat& frame8u, cv::Mat& dst) {
  cv::Mat frame;
  frame8u.convertTo(frame, CV_32F, 1.0 / 255);

  // Decompose image to 3 Lab channels
  cv::Mat frameLab, frameLabCn[3];
  cv::cvtColor(frame, frameLab, cv::COLOR_BGR2Lab);
  cv::split(frameLab, frameLabCn);

  // Generate progressively smoother versions of the lightness channel
  cv::Mat layer0 = frameLabCn[0]; // first channel is original lightness
  cv::Mat layer1, layer2;
  cv::ximgproc::dtFilter(layer0, layer0, layer1, g_sigmaSpatial, g_sigmaColor, DTF_IC);
  cv::ximgproc::dtFilter(layer1, layer1, layer2, 2 * g_sigmaSpatial, g_sigmaColor, DTF_IC);

  // Compute detail layers
  cv::Mat detailLayer1 = layer0 - layer1;
  cv::Mat detailLayer2 = layer1 - layer2;

  double cBase = g_contrastBase / 100.0;
  double cDetails1 = g_detailsLevel / 100.0;
  double cDetails2 = 2.0 - g_detailsLevel / 100.0;

  // Generate lightness
  double meanLigtness = mean(frameLabCn[0])[0];
  frameLabCn[0] = cBase * (layer2 - meanLigtness) + meanLigtness; // fit contrast of base (most blurred) layer
  frameLabCn[0] += cDetails1 * detailLayer1;                      // add weighted sum of detail layers to new lightness
  frameLabCn[0] += cDetails2 * detailLayer2;                      //

  // Update new lightness
  cv::merge(frameLabCn, 3, frameLab);
  cv::cvtColor(frameLab, frame, cv::COLOR_Lab2BGR);
  frame.convertTo(dst, CV_8U, 255);
}

void
changeModeCallback(int state, void* filter) {
  if(state == 1)
    g_filterOp = (FilteringOperation)filter;
}

void
changeNumberOfCpuCallback(int count, void*) {
  count = std::max(1, count);
  g_numberOfCPUs = count;
}

// divide screen on two parts: srcFrame and processed Frame
void
splitScreen(const cv::Mat& rawFrame, cv::Mat& outputFrame, cv::Mat& srcFrame, cv::Mat& processedFrame) {
  int h = rawFrame.rows;
  int w = rawFrame.cols;
  int cn = rawFrame.channels();

  outputFrame.create(h, 2 * w, CV_MAKE_TYPE(CV_8U, cn));
  srcFrame = outputFrame(cv::Range::all(), cv::Range(0, w));
  processedFrame = outputFrame(cv::Range::all(), cv::Range(w, 2 * w));
  rawFrame.convertTo(srcFrame, srcFrame.type());
}

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    cerr << "Capture device was not found" << endl;
    return -1;
  }

  cv::namedWindow("Demo");
  cv::displayOverlay("Demo", "Press Ctrl+P to show property window", 5000);

  // Thread trackbar
  cv::createTrackbar("Threads", cv::String(), &g_numberOfCPUs, cv::getNumberOfCPUs(), changeNumberOfCpuCallback);

  // Buttons to choose different modes
  cv::createButton("Mode Details Enhancement", changeModeCallback, (void*)filterDetailEnhancement, cv::QT_RADIOBOX, true);
  cv::createButton("Mode Stylizing", changeModeCallback, (void*)filterStylize, cv::QT_RADIOBOX, false);
  cv::createButton("Mode Blurring", changeModeCallback, (void*)filterBlurring, cv::QT_RADIOBOX, false);
  cv::createButton("Mode DoNothing", changeModeCallback, (void*)filterDoNothing, cv::QT_RADIOBOX, false);

  // sliders for Details Enhancement mode
  g_filterOp = filterDetailEnhancement; // set Details Enhancement as default filter
  cv::createTrackbar("Detail contrast", cv::String(), &g_contrastBase, 200);
  cv::createTrackbar("Detail level", cv::String(), &g_detailsLevel, 200);

  // sliders for Stylizing mode
  cv::createTrackbar("Style gamma", cv::String(), &g_edgesGamma, 300);

  // sliders for every mode
  cv::createTrackbar("Sigma Spatial", cv::String(), &g_sigmaSpatial, 200);
  cv::createTrackbar("Sigma Color", cv::String(), &g_sigmaColor, 200);

  cv::Mat rawFrame, outputFrame;
  cv::Mat srcFrame, processedFrame;

  for(;;) {
    do { cap >> rawFrame; } while(rawFrame.empty());

    cv::setNumThreads(g_numberOfCPUs); // speedup filtering

    splitScreen(rawFrame, outputFrame, srcFrame, processedFrame);
    g_filterOp(srcFrame, processedFrame);

    cv::imshow("Demo", outputFrame);

    if(cv::waitKey(1) == 27)
      break;
  }

  return 0;
}
