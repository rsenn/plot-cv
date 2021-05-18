#include <opencv2/opencv.hpp>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  // Read source image.
  cv::Mat im_src = cv::imread("book2.jpg");
  // Four corners of the book in source image
  vector<cv::Point2f> pts_src;
  pts_src.push_back(cv::Point2f(141, 131));
  pts_src.push_back(cv::Point2f(480, 159));
  pts_src.push_back(cv::Point2f(493, 630));
  pts_src.push_back(cv::Point2f(64, 601));

  // Read destination image.
  cv::Mat im_dst = cv::imread("book1.jpg");
  // Four corners of the book in destination image.
  vector<cv::Point2f> pts_dst;
  pts_dst.push_back(cv::Point2f(318, 256));
  pts_dst.push_back(cv::Point2f(534, 372));
  pts_dst.push_back(cv::Point2f(316, 670));
  pts_dst.push_back(cv::Point2f(73, 473));

  // Calculate Homography
  cv::Mat h = cv::findHomography(pts_src, pts_dst);

  // Output image
  cv::Mat im_out;
  // Warp source image to destination based on homography
  cv::warpPerspective(im_src, im_out, h, im_dst.size());

  // Display images
  cv::imshow("Source Image", im_src);
  cv::imshow("Destination Image", im_dst);
  cv::imshow("Warped Source Image", im_out);

  cv::waitKey(0);
}
