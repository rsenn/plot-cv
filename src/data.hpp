#ifndef DATA_HPP
#define DATA_HPP

#pragma once
#include <opencv2/highgui/highgui.hpp>

// using namespace cv;
using namespace std;

#include "polygon.hpp"
enum SegmentationMode { BALLOON_MODE, CURV_MODE, ALL_MODE };

class Data {

public:
  Data(cv::Mat& Image);
  cv::Mat image;     // Image originale
  cv::Mat bwImage;   // Image convertie en noir et blanc.
  cv::Mat gradient;  // Norme du gradient
  cv::Mat gGradient; // g(Norme du gradient)
  cv::Mat gx;        // gradient en x de g
  cv::Mat gy;        // gradient en y de g
  Polygon polygon;   // polygone lié à l'image

  void draw_next_step(double step, const cv::Mat& Image, SegmentationMode mode);
  void find_contour(double step, SegmentationMode mode);
  bool is_valid_point(cv::Point2d p);
  // Modigie le polygone en utilisant une descente de gradient et compris dans
  // les bords de l'image.
};

#endif // defined DATA_HPP