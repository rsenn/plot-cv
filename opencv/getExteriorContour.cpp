/*
 * @ Brief: this function takes in a cv::Mat image and returns the exterior contour of object
 * @ Description:
 *   Binarize -> cv::dilate -> edge -> findContour -> select contour that has largest area
 */

#include <getExteriorContour.hpp>

// #define DEBUG_EXTERIOR

bool
cmpArea(vector<cv::Point> a, vector<Point> b) {
  double i = fabs(cv::contourArea(cv::Mat(a)));
  double j = fabs(cv::contourArea(cv::Mat(b)));
  return (i > j);
}

void
getExteriorContour(cv::Mat src, vector<cv::Point>& contour) {

  /* parameters */
  int dil = 13;

  // binarize image
  cv::Mat image_bin;
  cv::threshold(src, image_bin, 0, 255, 8);
#ifdef DEBUG_EXTERIOR
  cv::namedWindow("image_bin", cv::WINDOW_NORMAL);
  cv::resizeWindow("image_bin", src.cols / 10, src.rows / 10);
  cv::imshow("image_bin", image_bin);
  cv::waitKey(0);
#endif

  // dilated binary image
  cv::Mat image_dil;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dil, dil));
  cv::dilate(image_bin, image_dil, element);
#ifdef DEBUG_EXTERIOR
  cv::namedWindow("image_dil", cv::WINDOW_NORMAL);
  cv::resizeWindow("image_dil", src.cols / 10, src.rows / 10);
  cv::imshow("image_dil", image_dil);
  cv::waitKey(0);
#endif

  // edge detection
  cv::Mat edged;
  cv::Canny(image_dil, edged, 100, 220);
#ifdef DEBUG_EXTERIOR
  cv::namedWindow("edged", cv::WINDOW_NORMAL);
  cv::resizeWindow("edged", src.cols / 10, src.rows / 10);
  cv::imshow("edged", edged);
  cv::waitKey(0);
#endif

  // find all contours
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(edged, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

  // Sort by area
  sort(contours.begin(), contours.end(), cmpArea);

  // The exterior contour
  contour = contours[0];
}
