/**
 * @file houghlines.cpp
 * @brief This program demonstrates cv::line finding with the Hough transform
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  // Declare the output variables
  cv::Mat dst, cdst, cdstP;

  //![load]
  const char* default_file = "sudoku.png";
  const char* filename = argc >= 2 ? argv[1] : default_file;

  // Loads an image
  cv::Mat src = cv::imread(cv::samples::findFile(filename), cv::IMREAD_GRAYSCALE);

  // Check if image is loaded fine
  if(src.empty()) {
    printf(" cv::Error opening image\n");
    printf(" Program Arguments: [image_name -- default %s] \n", default_file);
    return -1;
  }
  //![load]

  //![edge_detection]
  // Edge detection
  cv::Canny(src, dst, 50, 200, 3);
  //![edge_detection]

  // Copy edges to the images that will display the results in BGR
  cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
  cdstP = cdst.clone();

  //![hough_lines]
  // Standard Hough Line Transform
  vector<cv::Vec2f> lines;                               // will hold the results of the detection
  cv::HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0); // runs the actual detection
  //![hough_lines]
  //![draw_lines]
  // Draw the lines
  for(size_t i = 0; i < lines.size(); i++) {
    float rho = lines[i][0], theta = lines[i][1];
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    cv::line(cdst, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
  }
  //![draw_lines]

  //![hough_lines_p]
  // Probabilistic Line Transform
  vector<cv::Vec4i> linesP;                                 // will hold the results of the detection
  cv::HoughLinesP(dst, linesP, 1, CV_PI / 180, 50, 50, 10); // runs the actual detection
  //![hough_lines_p]
  //![draw_lines_p]
  // Draw the lines
  for(size_t i = 0; i < linesP.size(); i++) {
    cv::Vec4i l = linesP[i];
    cv::line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
  }
  //![draw_lines_p]

  //![cv::imshow]
  // Show results
  cv::imshow("Source", src);
  cv::imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
  cv::imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
  //![cv::imshow]

  //![exit]
  // Wait and Exit
  cv::waitKey();
  return 0;
  //![exit]
}
