/*
 * EEL6562 Project
 * Real Time Object Recognition using SURF
 *
 *  Created on: Nov 15, 2013
 *      Author: Frank
 */

// Include statements
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

// Name spaces used
//using namespace cv;
using namespace std;

int
main() {
  // turn performance analysis functions on if testing = true
  bool testing = false;
  double t; // timing variable

  // load training image
  cv::Mat object = cv::imread("C:/School/Image Processing/book.jpg");
  if(!object.data) {
    cout << "Can't open image";
    return -1;
  }
  cv::namedWindow("Good Matches", cv::WINDOW_AUTOSIZE);

  // SURF Detector, and descriptor parameters
  int minHess = 3000;
  vector<KeyPoint> kpObject, kpImage;
  cv::Mat desObject, desImage;

  // Performance measures calculations for report
  if(testing) {
    cout << object.rows << " " << object.cols << endl;

    // calculate integral image
    cv::Mat iObject;
    cv::integral(object, iObject);
    cv::imshow("Good Matches", iObject);
    cv::imwrite("C:/School/Image Processing/IntegralImage.jpg", iObject);
    cv::waitKey(0);

    // calculate number of interest points, computation time as f(minHess)
    int minHessVector[] = {100,  500,  1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000,
                           5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000};
    int minH;
    std::ofstream file;
    file.open("C:/School/Image Processing/TimingC.csv", std::ofstream::out);
    for(int i = 0; i < 20; i++) {
      minH = minHessVector[i];
      t = (double)cv::getTickCount();
      SurfFeatureDetector detector(minH);
      detector.detect(object, kpObject);
      t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      file << minHess << "," << kpObject.size() << "," << t << ",";
      cout << t << " " << kpObject.size() << " " << desObject.size() << endl;

      t = (double)cv::getTickCount();
      SurfDescriptorExtractor extractor;
      extractor.compute(object, kpObject, desObject);
      t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      file << t << endl;
    }
    file.close();

    // Display keypoints on training image
    cv::Mat interestPointObject = object;
    for(unsigned int i = 0; i < kpObject.size(); i++) {
      if(kpObject[i].octave) {
        cv::circle(interestPointObject, kpObject[i].pt, kpObject[i].size, 0);
        string octaveS;
        switch(kpObject[i].octave) {
          case 0: octaveS = "0"; break;
          case 1: octaveS = '1'; break;
          case 2: octaveS = '2'; break;
          default: break;
        }
        cv::putText(
            interestPointObject, octaveS, kpObject[i].pt, FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 250), 1, cv::LINE_AA);
      }
    }
    cv::imshow("Good Matches", interestPointObject);
    cv::imwrite("C:/School/Image Processing/bookIP2.jpg", interestPointObject);
    cv::waitKey(0);
  }

  // SURF Detector, and descriptor parameters, match object initialization
  minHess = 2000;
  SurfFeatureDetector detector(minHess);
  detector.detect(object, kpObject);
  SurfDescriptorExtractor extractor;
  extractor.compute(object, kpObject, desObject);
  FlannBasedMatcher matcher;

  // Initialize video and display window
  cv::VideoCapture cap(1); // camera 1 is webcam
  if(!cap.isOpened())
    return -1;

  // Object corner points for plotting box
  vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cv::Point(0, 0);
  obj_corners[1] = cv::Point(object.cols, 0);
  obj_corners[2] = cv::Point(object.cols, object.rows);
  obj_corners[3] = cv::Point(0, object.rows);

  // video loop
  char escapeKey = 'k';
  double frameCount = 0;
  float thresholdMatchingNN = 0.7;
  unsigned int thresholdGoodMatches = 4;
  unsigned int thresholdGoodMatchesV[] = {4, 5, 6, 7, 8, 9, 10};

  for(int j = 0; j < 7; j++) {
    thresholdGoodMatches = thresholdGoodMatchesV[j];
    // thresholdGoodMatches=8;
    cout << thresholdGoodMatches << endl;

    if(true) {
      t = (double)cv::getTickCount();
    }

    while(escapeKey != 'q') {
      frameCount++;
      cv::Mat frame;
      cv::Mat image;
      cap >> frame;
      cv::cvtColor(frame, image, cv::COLOR_RGB2GRAY);

      cv::Mat des_image, img_matches, H;
      vector<KeyPoint> kp_image;
      vector<vector<DMatch>> matches;
      vector<DMatch> good_matches;
      vector<cv::Point2f> obj;
      vector<cv::Point2f> scene;
      vector<cv::Point2f> scene_corners(4);

      detector.detect(image, kp_image);
      extractor.compute(image, kp_image, des_image);
      matcher.knnMatch(desObject, des_image, matches, 2);

      for(int i = 0; i < min(des_image.rows - 1, (int)matches.size()); i++) // THIS LOOP IS SENSITIVE TO SEGFAULTS
      {
        if((matches[i][0].distance < thresholdMatchingNN * (matches[i][1].distance)) &&
           ((int)matches[i].size() <= 2 && (int)matches[i].size() > 0)) {
          good_matches.push_back(matches[i][0]);
        }
      }

      // if (good_matches.size()<1)
      //	good_matches.cv::resize(0,cv::DMatch);

      // Draw only "good" matches
      drawMatches(object,
                  kpObject,
                  image,
                  kp_image,
                  good_matches,
                  img_matches,
                  cv::Scalar::all(-1),
                  cv::Scalar::all(-1),
                  vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

      if(good_matches.size() >= thresholdGoodMatches) {

        // Display that the object is found
        cv::putText(img_matches,
                "Object Found",
                cv::Point(10, 50),
                FONT_HERSHEY_COMPLEX_SMALL,
                2,
                cv::Scalar(0, 0, 250),
                1,
                cv::LINE_AA);
        for(unsigned int i = 0; i < good_matches.size(); i++) {
          // Get the keypoints from the good matches
          obj.push_back(kpObject[good_matches[i].queryIdx].pt);
          scene.push_back(kp_image[good_matches[i].trainIdx].pt);
        }

        H = cv::findHomography(obj, scene, cv::RANSAC);

        perspectiveTransform(obj_corners, scene_corners, H);

        // Draw lines between the corners (the mapped object in the scene image )
        cv::line(img_matches,
             scene_corners[0] + cv::Point2f(object.cols, 0),
             scene_corners[1] + cv::Point2f(object.cols, 0),
             cv::Scalar(0, 255, 0),
             4);
        cv::line(img_matches,
             scene_corners[1] + cv::Point2f(object.cols, 0),
             scene_corners[2] + cv::Point2f(object.cols, 0),
             cv::Scalar(0, 255, 0),
             4);
        cv::line(img_matches,
             scene_corners[2] + cv::Point2f(object.cols, 0),
             scene_corners[3] + cv::Point2f(object.cols, 0),
             cv::Scalar(0, 255, 0),
             4);
        cv::line(img_matches,
             scene_corners[3] + cv::Point2f(object.cols, 0),
             scene_corners[0] + cv::Point2f(object.cols, 0),
             cv::Scalar(0, 255, 0),
             4);
      } else {
        cv::putText(img_matches, "", cv::Point(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 3, cv::Scalar(0, 0, 250), 1, cv::LINE_AA);
      }

      // Show detected matches
      cv::imshow("Good Matches", img_matches);
      escapeKey = cv::waitKey(10);
      // cv::imwrite("C:/School/Image Processing/bookIP3.jpg", img_matches);

      if(frameCount > 10)
        escapeKey = 'q';
    }

    // average frames per second
    if(true) {
      t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      cout << t << " " << frameCount / t << endl;
      cv::waitKey(0);
    }

    frameCount = 0;
    escapeKey = 'a';
  }

  // Release camera and exit
  cap.release();
  return 0;
}
