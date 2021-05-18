/*
 * @ Brief: this function finds corners of a RECTANGLE
 * @ Description:
 *   Get exterior contour -> Polygon fitting -> Find corners
 */
#include <Measure_Origins.hpp>
#include <getExteriorContour.hpp>

//#define DEBUG

void
Measure_Origins(cv::Mat& src) {

  // get The exterior contour
  vector<cv::Point> contour;
  cv::Mat edged;
  getExteriorContour(src, contour);
  cv::Rect box = cv::boundingRect(cv::Mat(contour));
  vector<cv::Point2f> rectCorners(4);
  rectCorners[0] = cv::Point2f(box.x, box.y);
  rectCorners[1] = cv::Point2f(box.x + box.width, box.y);
  rectCorners[2] = cv::Point2f(box.x, box.y + box.height);
  rectCorners[3] = cv::Point2f(box.x + box.width, box.y + box.height);

#ifdef DEBUG
  cout << "Box axis:" << endl;
  cout << "bottom left = "
       << "(" << box.x << " , " << box.y << ")" << endl;
  cout << "bottom right = "
       << "(" << box.x + box.width << " , " << box.y << ")" << endl;
  cout << "top left = "
       << "(" << box.x << " , " << box.y + box.height << ")" << endl;
  cout << "top right = "
       << "(" << box.x + box.width << " , " << box.y + box.height << ")" << endl;
#endif

  // draw corners
  cv::Mat img_corners;
  cv::cvtColor(src, img_corners, cv::COLOR_GRAY2BGR);

  for(int i = 0; i < rectCorners.size(); i++) { cv::circle(img_corners, rectCorners[i], 50, cv::Scalar(0, 0, 255), 8, 8, 0); }
  cv::namedWindow("corners", WINDOW_NORMAL);
  cv::resizeWindow("corners", src.cols / 10, src.rows / 10);
  cv::imshow("corners", img_corners);
  cv::imwrite("./output/corners.jpg", img_corners);
  cv::waitKey(0);
}