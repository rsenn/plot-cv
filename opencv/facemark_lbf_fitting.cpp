/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.
                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)
Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.

This file was part of GSoC Project: Facemark API for OpenCV
Final report: https://gist.github.com/kurnianggoro/74de9121e122ad0bd825176751d47ecc
Student: Laksono Kurnianggoro
Mentor: Delia Passalacqua
*/

/*----------------------------------------------
 * Usage:
 * facemark_lbf_fitting <face_cascade_model> <lbf_model> <video_name>
 *
 * example:
 * facemark_lbf_fitting ../face_cascade.xml ../LBF.model ../video.mp4
 *
 * note: do not forget to provide the LBF_MODEL and DETECTOR_MODEL
 * the model are available at opencv_contrib/modules/cv::face/data/
 *--------------------------------------------------*/

#include <stdio.h>
#include <ctime>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/face.hpp>

using namespace std;
// using namespace cv;
// using namespace cv::face;

static bool myDetector(cv::InputArray image, cv::OutputArray ROIs, cv::CascadeClassifier* face_cascade);
static bool parseArguments(int argc, char** argv, cv::String& cascade, cv::String& model, cv::String& video);

int
main(int argc, char** argv) {
  cv::String cascade_path, model_path, images_path, video_path;
  if(!parseArguments(argc, argv, cascade_path, model_path, video_path))
    return -1;

  cv::CascadeClassifier face_cascade;
  face_cascade.load(cascade_path);

  cv::face::FacemarkLBF::Params params;
  params.model_filename = model_path;
  params.cascade_face = cascade_path;

  cv::Ptr<cv::face::FacemarkLBF> facemark = cv::face::FacemarkLBF::create(params);
  facemark->setFaceDetector((cv::face::FN_FaceDetector)myDetector, &face_cascade);
  facemark->loadModel(params.model_filename.c_str());

  cv::VideoCapture capture(video_path);
  cv::Mat frame;

  if(!capture.isOpened()) {
    printf("cv::Error when reading vide\n");
    return 0;
  }

  cv::Mat img;
  cv::String text;
  char buff[255];
  double fittime;
  int nfaces;
  std::vector<cv::Rect> rects, rects_scaled;
  std::vector<std::vector<cv::Point2f>> landmarks;
  cv::CascadeClassifier cc(params.cascade_face.c_str());
  cv::namedWindow("w", 1);
  for(;;) {
    capture >> frame;
    if(frame.empty())
      break;

    double __time__ = (double)cv::getTickCount();

    float scale = (float)(400.0 / frame.cols);
    cv::resize(frame, img, cv::Size((int)(frame.cols * scale), (int)(frame.rows * scale)), 0, 0, cv::INTER_LINEAR_EXACT);

    facemark->getFaces(img, rects);
    rects_scaled.clear();

    for(int j = 0; j < (int)rects.size(); j++) {
      rects_scaled.push_back(cv::Rect(
          (int)(rects[j].x / scale), (int)(rects[j].y / scale), (int)(rects[j].width / scale), (int)(rects[j].height / scale)));
    }
    rects = rects_scaled;
    fittime = 0;
    nfaces = (int)rects.size();
    if(rects.size() > 0) {
      double newtime = (double)cv::getTickCount();

      facemark->fit(frame, rects, landmarks);

      fittime = ((cv::getTickCount() - newtime) / cv::getTickFrequency());
      for(int j = 0; j < (int)rects.size(); j++) {
        landmarks[j] = cv::Mat(cv::Mat(landmarks[j]));
        cv::face::drawFacemarks(frame, landmarks[j], cv::Scalar(0, 0, 255));
      }
    }

    double fps = (cv::getTickFrequency() / (cv::getTickCount() - __time__));
    sprintf(buff, "faces: %i %03.2f fps, fit:%03.0f ms", nfaces, fps, fittime * 1000);
    text = buff;
    cv::putText(frame, text, cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar::all(255), 2, 8);

    cv::imshow("w", frame);
    cv::waitKey(1); // waits to display frame
  }
  cv::waitKey(0); // key press to close window
}

bool
myDetector(cv::InputArray image, cv::OutputArray faces, cv::CascadeClassifier* face_cascade) {
  cv::Mat gray;

  if(image.channels() > 1)
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else
    gray = image.getMat().clone();

  cv::equalizeHist(gray, gray);

  std::vector<cv::Rect> faces_;
  face_cascade->detectMultiScale(gray, faces_, 1.4, 2, cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
  cv::Mat(faces_).copyTo(faces);
  return true;
}

bool
parseArguments(int argc, char** argv, cv::String& cascade, cv::String& model, cv::String& video) {
  const cv::String keys = "{ @c cascade         |      | (required) path to the cascade model file for the cv::face "
                          "detector }"
                          "{ @m model           |      | (required) path to the trained model }"
                          "{ @v video           |      | (required) path input video}"
                          "{ help h usage ?     |      | facemark_lbf_fitting -cascade -model -video [-t]\n"
                          " example: facemark_lbf_fitting ../face_cascade.xml ../LBF.model ../video.mp4}";
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about("hello");

  if(parser.has("help")) {
    parser.printMessage();
    return false;
  }

  cascade = cv::String(parser.get<cv::String>("cascade"));
  model = cv::String(parser.get<string>("model"));
  video = cv::String(parser.get<string>("video"));

  if(cascade.empty() || model.empty() || video.empty()) {
    std::cerr << "one or more required arguments are not found" << '\n';
    cout << "cascade : " << cascade.c_str() << endl;
    cout << "model : " << model.c_str() << endl;
    cout << "video : " << video.c_str() << endl;
    parser.printMessage();
    return false;
  }

  return true;
}
