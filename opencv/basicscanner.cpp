#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <simple_svg_1.0.0.hpp>

// using namespace cv;
using namespace std;

cv::Mat src;
cv::Mat src_gray;
cv::Mat resize_img;
int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);
int largestContour = -1;
double max_svg_width = 1200; // pixels
double max_svg_height = 900; // pixels
vector<cv::Point> bigContour;

bool showDiagnostics = false;
bool showWindow = false;

/// Function header
void thresh_callback(int, void*);

// finds the largest contour and stores the bigContour and stores it index which are both global
// variables.
void
filter_contours(vector<vector<cv::Point>> contours_un) {
  double maxArea = 0.0;
  for(int i = 0; i < contours_un.size(); i++) {
    double area = cv::contourArea(contours_un[i]);
    if(showDiagnostics) {
      std::cout << "Area: " + to_string(area) + "Index: " + to_string(i) << std::endl;
    }
    if(area > maxArea) {
      maxArea = area;
      largestContour = i;
    }
  }
  bigContour = contours_un.at(largestContour);
}

void
export_svg(vector<cv::Point> contour_arg, string output_file) {

  svg::Dimensions dimensions(max_svg_width, max_svg_height);
  svg::Document doc(output_file, svg::Layout(dimensions, svg::Layout::TopLeft));

  svg::LineChart chart(5.0);
  svg::Polyline polyline(svg::Stroke(1, svg::Color(255, 0, 0)));
  for(int i = 0; i < contour_arg.size(); i++) {
    svg::Point temp = svg::Point(contour_arg.at(i).x, contour_arg.at(i).y);
    polyline << temp;
  }
  doc << polyline;
  doc.save();

  std::cout << "EXPORTED SVG" << std::endl;
}

/** @function main */

// Format: imgfile parameter1 parameter2
int
main(int argc, char** argv) {
  if(argc > 2) {
    if(std::strcmp(argv[2], "-S") == 0) {
      showWindow = true;
    }

    if(argc > 3 && std::strcmp(argv[3], "-D") == 0) {
      showDiagnostics = true;
    }
  }

  /// Load source image and convert it to gray

  src = cv::imread(argv[1], 1);

  if(src.empty()) {
    std::cout << "failed to open image file. Make sure img exists and command is in correct cv::format." << std::endl;
  } else {

    cv::Size dsize2 = cv::Size(round(.35 * src.cols), round(.35 * src.rows));
    cv::resize(src, resize_img, dsize2);

    src = resize_img;
    /// Convert image to gray and cv::blur it
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    cv::blur(src_gray, src_gray, cv::Size(3, 3));

    /// Create Window
    char* source_window = "Source";
    if(showWindow) {
      cv::namedWindow(source_window, cv::WINDOW_AUTOSIZE);
      cv::imshow(source_window, src);
    }
    thresh_callback(0, 0);
    export_svg(bigContour, "contour.svg");

    if(showWindow) {
      while(cv::waitKey(1) != '\x80') {}
    }
  }
  return (0);
}

/** @function thresh_callback */
void
thresh_callback(int, void*) {
  cv::Mat canny_output;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;

  /// Detect edges using canny
  cv::Canny(src_gray, canny_output, thresh, thresh * 2, 3);
  /// Find contours
  cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Draw contours
  filter_contours(contours);
  if(showDiagnostics) {
    std::cout << "largest Contour Index: " + to_string(largestContour) << std::endl;
    std::cout << "largest Contour Area: " + to_string(cv::contourArea(bigContour)) << std::endl;
  }
  cv::Mat drawing = cv::Mat::zeros(canny_output.size(), CV_8UC3);
  cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
  cv::drawContours(drawing, contours, largestContour, color, 2, 8, hierarchy, 0, cv::Point());

  if(showDiagnostics) {
    for(int i = 0; i < contours.size(); i++) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    }
  }

  /// Show in a window
  if(showWindow) {
    cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
    cv::imshow("Contours", drawing);
  }
}
