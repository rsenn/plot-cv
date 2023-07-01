/**
 * @file houghcircles.cpp
 * @brief This program demonstrates cv::circle finding with the Hough transform
 */
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  //![load]
  const char* filename = argc >= 2 ? argv[1] : "smarties.png";

  // Loads an image
  cv::Mat src = cv::imread(cv::samples::findFile(filename), cv::IMREAD_COLOR);

  // Check if image is loaded fine
  if(src.empty()) {
    printf(" cv::Error opening image\n");
    printf(" Program Arguments: [image_name -- default %s] \n", filename);
    return EXIT_FAILURE;
  }
  //![load]

  //![convert_to_gray]
  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  //![convert_to_gray]

  //![reduce_noise]
  cv::medianBlur(gray, gray, 5);
  //![reduce_noise]

  //![houghcircles]
  vector<cv::Vec3f> circles;
  cv::HoughCircles(gray,
                   circles,
                   HOUGH_GRADIENT,
                   1,
                   gray.rows / 16, // change this value to detect circles with different distances to each other
                   100,
                   30,
                   1,
                   30 // change the last two parameters
                      // (min_radius & max_radius) to detect larger circles
  );
  //![houghcircles]

  //![draw]
  for(size_t i = 0; i < circles.size(); i++) {
    Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // cv::circle center
    cv::circle(src, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    // cv::circle outline
    int radius = c[2];
    cv::circle(src, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
  }
  //![draw]

  //![display]
  cv::imshow("detected circles", src);
  cv::waitKey();
  //![display]

  return EXIT_SUCCESS;
}
